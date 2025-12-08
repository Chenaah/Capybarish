"""
Command Line Interface for Capybarish.

Provides CLI commands for:
- Code generation from .cpy schema files
- Arduino library installation
- Project scaffolding

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0.
"""

import argparse
import shutil
import sys
from pathlib import Path
from typing import List, Optional


def get_arduino_library_path() -> Path:
    """Get the path to the bundled Arduino library."""
    # The Arduino library is packaged with the Python package
    package_dir = Path(__file__).parent
    arduino_dir = package_dir.parent / "arduino"
    
    if not arduino_dir.exists():
        # Try alternative location (installed package)
        import capybarish
        package_root = Path(capybarish.__file__).parent.parent
        arduino_dir = package_root / "arduino"
    
    return arduino_dir


def cmd_arduino_install(args: argparse.Namespace) -> int:
    """Install Arduino library to specified path."""
    arduino_src = get_arduino_library_path()
    
    if not arduino_src.exists():
        print(f"Error: Arduino library not found at {arduino_src}")
        print("The Arduino library may not be bundled with this installation.")
        return 1
    
    # Determine destination
    if args.path:
        dest = Path(args.path)
    else:
        # Try to find default Arduino libraries folder
        home = Path.home()
        possible_paths = [
            home / "Arduino" / "libraries",
            home / "Documents" / "Arduino" / "libraries",
            home / ".arduino15" / "libraries",
        ]
        
        dest = None
        for p in possible_paths:
            if p.exists():
                dest = p
                break
        
        if dest is None:
            print("Error: Could not find Arduino libraries folder.")
            print("Please specify the path with --path")
            return 1
    
    dest = dest / "Capybarish"
    
    # Check if already exists
    if dest.exists():
        if args.force:
            print(f"Removing existing installation at {dest}")
            shutil.rmtree(dest)
        else:
            print(f"Error: {dest} already exists. Use --force to overwrite.")
            return 1
    
    # Copy library
    print(f"Installing Arduino library to {dest}")
    shutil.copytree(arduino_src, dest)
    
    print("Installation complete!")
    print(f"\nLibrary installed to: {dest}")
    print("\nTo use in your Arduino/PlatformIO project:")
    print('  #include <capybarish.h>')
    
    return 0


def cmd_generate(args: argparse.Namespace) -> int:
    """Generate code from schema file."""
    from .codegen import CodeGenerator
    
    schema_file = Path(args.schema)
    if not schema_file.exists():
        print(f"Error: Schema file not found: {schema_file}")
        return 1
    
    output_dir = Path(args.output) if args.output else Path(".")
    
    gen_python = args.python or args.all or (not args.cpp)
    gen_cpp = args.cpp or args.all
    
    print(f"Parsing schema: {schema_file}")
    
    try:
        generator = CodeGenerator()
        files = generator.generate(
            str(schema_file),
            str(output_dir),
            python=gen_python,
            cpp=gen_cpp,
            cpp_header_only=not args.cpp_source,
        )
        
        print(f"\nGenerated {len(files)} file(s):")
        for f in files:
            print(f"  - {f}")
        
        return 0
    except Exception as e:
        print(f"Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def cmd_validate(args: argparse.Namespace) -> int:
    """Validate a schema file."""
    from .codegen import SchemaParser
    
    schema_file = Path(args.schema)
    if not schema_file.exists():
        print(f"Error: Schema file not found: {schema_file}")
        return 1
    
    try:
        parser = SchemaParser()
        schema = parser.parse_file(str(schema_file))
        
        print(f"Schema: {schema_file}")
        print(f"Package: {schema.package or '(default)'}")
        print(f"Messages: {len(schema.messages)}")
        print()
        
        for msg_name, msg in schema.messages.items():
            size = msg.get_size(schema.messages)
            fmt = msg.get_struct_format(schema.messages)
            print(f"  {msg_name}:")
            print(f"    Size: {size} bytes")
            print(f"    Format: {fmt}")
            print(f"    Fields: {len(msg.fields)}")
            for field in msg.fields:
                arr_suffix = f"[{field.array_size}]" if field.is_array else ""
                nested_marker = " (nested)" if field.is_nested else ""
                print(f"      - {field.type_name}{arr_suffix} {field.name}{nested_marker}")
            print()
        
        print("Schema is valid!")
        return 0
    except Exception as e:
        print(f"Validation error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def cmd_init(args: argparse.Namespace) -> int:
    """Initialize a new schema file with example content."""
    output = Path(args.output) if args.output else Path("messages.cpy")
    
    if output.exists() and not args.force:
        print(f"Error: {output} already exists. Use --force to overwrite.")
        return 1
    
    template = '''# Capybarish Message Schema
# Generated template - customize for your application
#
# Usage:
#   capybarish-gen --python --cpp {filename}

package myproject

# Example command message (server -> device)
message Command:
    float32 target           # Target value
    float32 velocity         # Target velocity
    int32 mode               # Operation mode
    int32 flags              # Control flags
    float32 timestamp        # Command timestamp

# Example status message (device -> server)  
message Status:
    int32 device_id          # Device identifier
    float32 position         # Current position
    float32 velocity         # Current velocity
    int32 error_code         # Error code (0 = OK)
    int32 timestamp          # Status timestamp (microseconds)
'''
    
    template = template.format(filename=output.name)
    
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w") as f:
        f.write(template)
    
    print(f"Created schema template: {output}")
    print("\nNext steps:")
    print(f"  1. Edit {output} to define your messages")
    print(f"  2. Generate code: capybarish-gen --python --cpp {output}")
    
    return 0


def create_parser() -> argparse.ArgumentParser:
    """Create the argument parser."""
    parser = argparse.ArgumentParser(
        prog="capybarish",
        description="Capybarish - Lightweight communication middleware for robotics",
    )
    parser.add_argument(
        "--version",
        action="version",
        version="%(prog)s 0.1.0",
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # arduino-install command
    arduino_parser = subparsers.add_parser(
        "arduino-install",
        help="Install Arduino/ESP32 library",
        description="Install the Capybarish Arduino library to your Arduino libraries folder",
    )
    arduino_parser.add_argument(
        "--path",
        help="Path to Arduino libraries folder (auto-detected if not specified)",
    )
    arduino_parser.add_argument(
        "--force", "-f",
        action="store_true",
        help="Overwrite existing installation",
    )
    
    # gen command
    gen_parser = subparsers.add_parser(
        "gen",
        aliases=["generate"],
        help="Generate code from schema file",
        description="Generate Python and/or C++ code from a .cpy schema file",
    )
    gen_parser.add_argument(
        "schema",
        help="Path to .cpy schema file",
    )
    gen_parser.add_argument(
        "--output", "-o",
        help="Output directory (default: current directory)",
    )
    gen_parser.add_argument(
        "--python", "-p",
        action="store_true",
        help="Generate Python code",
    )
    gen_parser.add_argument(
        "--cpp", "-c",
        action="store_true",
        help="Generate C++ code",
    )
    gen_parser.add_argument(
        "--all", "-a",
        action="store_true",
        help="Generate both Python and C++ code",
    )
    gen_parser.add_argument(
        "--cpp-source",
        action="store_true",
        help="Generate separate .cpp source file (default: header-only)",
    )
    gen_parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output",
    )
    
    # validate command
    validate_parser = subparsers.add_parser(
        "validate",
        help="Validate a schema file",
        description="Parse and validate a .cpy schema file without generating code",
    )
    validate_parser.add_argument(
        "schema",
        help="Path to .cpy schema file",
    )
    validate_parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Verbose output",
    )
    
    # init command
    init_parser = subparsers.add_parser(
        "init",
        help="Create a new schema file template",
        description="Initialize a new .cpy schema file with example content",
    )
    init_parser.add_argument(
        "--output", "-o",
        help="Output filename (default: messages.cpy)",
    )
    init_parser.add_argument(
        "--force", "-f",
        action="store_true",
        help="Overwrite existing file",
    )
    
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point for CLI."""
    parser = create_parser()
    args = parser.parse_args(argv)
    
    if args.command is None:
        parser.print_help()
        return 0
    
    if args.command == "arduino-install":
        return cmd_arduino_install(args)
    elif args.command in ("gen", "generate"):
        return cmd_generate(args)
    elif args.command == "validate":
        return cmd_validate(args)
    elif args.command == "init":
        return cmd_init(args)
    else:
        parser.print_help()
        return 1


if __name__ == "__main__":
    sys.exit(main())
