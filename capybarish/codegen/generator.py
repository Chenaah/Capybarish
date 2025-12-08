"""
Main Code Generator interface for Capybarish.

Provides a unified interface for generating code in multiple languages
from .cpy schema files.

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0.
"""

from pathlib import Path
from typing import List, Optional

from .parser import SchemaParser, SchemaDef
from .python_gen import PythonGenerator
from .cpp_gen import CppGenerator


class CodeGenerator:
    """
    Main code generation interface.
    
    Supports generating Python and C++ code from Capybarish schema files.
    
    Example:
        >>> gen = CodeGenerator()
        >>> gen.generate("schemas/messages.cpy", output_dir="generated/",
        ...              python=True, cpp=True)
    """
    
    def __init__(self):
        self.parser = SchemaParser()
    
    def parse(self, schema_file: str) -> SchemaDef:
        """Parse a schema file."""
        return self.parser.parse_file(schema_file)
    
    def generate(
        self,
        schema_file: str,
        output_dir: str = ".",
        python: bool = True,
        cpp: bool = True,
        cpp_header_only: bool = True,
    ) -> List[str]:
        """
        Generate code from a schema file.
        
        Args:
            schema_file: Path to .cpy schema file
            output_dir: Output directory for generated files
            python: Generate Python code
            cpp: Generate C++ code
            cpp_header_only: Generate header-only C++ (no .cpp file)
        
        Returns:
            List of generated file paths
        """
        schema = self.parse(schema_file)
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        generated_files = []
        package = schema.package or "messages"
        
        if python:
            py_path = output_path / f"{package}_messages.py"
            gen = PythonGenerator(schema)
            gen.write_file(str(py_path))
            generated_files.append(str(py_path))
            print(f"Generated: {py_path}")
        
        if cpp:
            hpp_path = output_path / f"{package}_messages.hpp"
            gen = CppGenerator(schema)
            gen.write_header(str(hpp_path))
            generated_files.append(str(hpp_path))
            print(f"Generated: {hpp_path}")
            
            if not cpp_header_only:
                cpp_path = output_path / f"{package}_messages.cpp"
                gen.write_source(str(cpp_path))
                generated_files.append(str(cpp_path))
                print(f"Generated: {cpp_path}")
        
        return generated_files
    
    def generate_from_string(
        self,
        schema_content: str,
        output_dir: str = ".",
        python: bool = True,
        cpp: bool = True,
    ) -> List[str]:
        """
        Generate code from schema content string.
        
        Args:
            schema_content: Schema definition as string
            output_dir: Output directory
            python: Generate Python code
            cpp: Generate C++ code
        
        Returns:
            List of generated file paths
        """
        schema = self.parser.parse_string(schema_content)
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        generated_files = []
        package = schema.package or "messages"
        
        if python:
            py_path = output_path / f"{package}_messages.py"
            gen = PythonGenerator(schema)
            gen.write_file(str(py_path))
            generated_files.append(str(py_path))
        
        if cpp:
            hpp_path = output_path / f"{package}_messages.hpp"
            gen = CppGenerator(schema)
            gen.write_header(str(hpp_path))
            generated_files.append(str(hpp_path))
        
        return generated_files


def generate_all(
    schema_file: str,
    output_dir: str = ".",
    python: bool = True,
    cpp: bool = True,
) -> List[str]:
    """
    Convenience function to generate all code from a schema file.
    
    Args:
        schema_file: Path to .cpy schema file
        output_dir: Output directory
        python: Generate Python code
        cpp: Generate C++ code
    
    Returns:
        List of generated file paths
    """
    gen = CodeGenerator()
    return gen.generate(schema_file, output_dir, python=python, cpp=cpp)
