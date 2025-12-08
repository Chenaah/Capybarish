"""
Schema Parser for Capybarish message definitions.

This module parses .cpy schema files that define message structures
for cross-platform communication between Python and C++/ESP32.

Schema Format Example:
    
    # Comment line
    package mypackage
    
    message ReceivedData:
        float32 target
        float32 target_vel
        float32 kp
        float32 kd
        int32 enable_filter
        int32 switch_
        int32 calibrate
        int32 restart
        float32 timestamp
    
    message IMUOrientation:
        float32 x
        float32 y
        float32 z
    
    message SentData:
        int32 module_id
        int32 timestamp
        IMUOrientation orientation  # Nested message

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0.
"""

import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple


class FieldType(Enum):
    """Supported field types for message definitions."""
    
    # Signed integers
    INT8 = "int8"
    INT16 = "int16"
    INT32 = "int32"
    INT64 = "int64"
    
    # Unsigned integers
    UINT8 = "uint8"
    UINT16 = "uint16"
    UINT32 = "uint32"
    UINT64 = "uint64"
    
    # Floating point
    FLOAT32 = "float32"
    FLOAT64 = "float64"
    
    # Aliases
    FLOAT = "float"      # alias for float32
    DOUBLE = "double"    # alias for float64
    INT = "int"          # alias for int32
    BOOL = "bool"        # stored as uint8
    
    # Special
    BYTE = "byte"        # alias for uint8
    CHAR = "char"        # alias for int8
    
    # Custom/nested message type (resolved during parsing)
    CUSTOM = "custom"


# Type mappings for code generation
TYPE_INFO = {
    # type: (python_type, cpp_type, struct_format, size_bytes)
    FieldType.INT8:    ("int", "int8_t", "b", 1),
    FieldType.INT16:   ("int", "int16_t", "h", 2),
    FieldType.INT32:   ("int", "int32_t", "i", 4),
    FieldType.INT64:   ("int", "int64_t", "q", 8),
    FieldType.UINT8:   ("int", "uint8_t", "B", 1),
    FieldType.UINT16:  ("int", "uint16_t", "H", 2),
    FieldType.UINT32:  ("int", "uint32_t", "I", 4),
    FieldType.UINT64:  ("int", "uint64_t", "Q", 8),
    FieldType.FLOAT32: ("float", "float", "f", 4),
    FieldType.FLOAT64: ("float", "double", "d", 8),
    FieldType.FLOAT:   ("float", "float", "f", 4),
    FieldType.DOUBLE:  ("float", "double", "d", 8),
    FieldType.INT:     ("int", "int32_t", "i", 4),
    FieldType.BOOL:    ("bool", "bool", "?", 1),
    FieldType.BYTE:    ("int", "uint8_t", "B", 1),
    FieldType.CHAR:    ("str", "char", "c", 1),
}

# Mapping from string to FieldType
TYPE_MAP = {t.value: t for t in FieldType if t != FieldType.CUSTOM}


@dataclass
class FieldDef:
    """Definition of a field in a message."""
    
    name: str
    type_name: str  # Original type name from schema
    field_type: FieldType
    array_size: Optional[int] = None  # None = scalar, int = fixed array
    comment: Optional[str] = None
    is_nested: bool = False  # True if this is a nested message type
    
    @property
    def is_array(self) -> bool:
        """Check if this field is an array."""
        return self.array_size is not None
    
    def get_type_info(self) -> Tuple[str, str, str, int]:
        """Get type information (python_type, cpp_type, struct_format, size)."""
        if self.is_nested:
            raise ValueError(f"Cannot get type info for nested type: {self.type_name}")
        return TYPE_INFO[self.field_type]


@dataclass
class MessageDef:
    """Definition of a message type."""
    
    name: str
    fields: List[FieldDef] = field(default_factory=list)
    comment: Optional[str] = None
    
    def get_struct_format(self, messages: Dict[str, "MessageDef"]) -> str:
        """Get the Python struct format string for this message."""
        format_parts = []
        for f in self.fields:
            if f.is_nested:
                # Recursively get nested message format
                nested_msg = messages.get(f.type_name)
                if nested_msg:
                    nested_format = nested_msg.get_struct_format(messages)
                    if f.is_array:
                        format_parts.append(nested_format * f.array_size)
                    else:
                        format_parts.append(nested_format)
            else:
                _, _, fmt, _ = f.get_type_info()
                if f.is_array:
                    format_parts.append(fmt * f.array_size)
                else:
                    format_parts.append(fmt)
        return "".join(format_parts)
    
    def get_size(self, messages: Dict[str, "MessageDef"]) -> int:
        """Calculate the total size of this message in bytes."""
        total = 0
        for f in self.fields:
            if f.is_nested:
                nested_msg = messages.get(f.type_name)
                if nested_msg:
                    size = nested_msg.get_size(messages)
                    total += size * (f.array_size or 1)
            else:
                _, _, _, size = f.get_type_info()
                total += size * (f.array_size or 1)
        return total


@dataclass
class SchemaDef:
    """Complete schema definition from a .cpy file."""
    
    package: Optional[str] = None
    messages: Dict[str, MessageDef] = field(default_factory=dict)
    imports: List[str] = field(default_factory=list)
    source_file: Optional[str] = None


class SchemaParser:
    """Parser for .cpy schema files."""
    
    # Regex patterns
    COMMENT_PATTERN = re.compile(r"#.*$")
    PACKAGE_PATTERN = re.compile(r"^\s*package\s+(\w+)\s*$")
    IMPORT_PATTERN = re.compile(r"^\s*import\s+([\"']?)(.+?)\1\s*$")
    MESSAGE_PATTERN = re.compile(r"^\s*message\s+(\w+)\s*:\s*$")
    FIELD_PATTERN = re.compile(
        r"^\s+(\w+)"               # type
        r"(?:\[(\d+)\])?"         # optional array size [N]
        r"\s+(\w+)"               # field name
        r"(?:\s*#\s*(.*))?$"      # optional comment
    )
    
    def __init__(self):
        self._schemas: Dict[str, SchemaDef] = {}
    
    def parse_file(self, filepath: str) -> SchemaDef:
        """Parse a .cpy schema file."""
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"Schema file not found: {filepath}")
        
        with open(path, "r") as f:
            content = f.read()
        
        return self.parse_string(content, source_file=str(path))
    
    def parse_string(self, content: str, source_file: Optional[str] = None) -> SchemaDef:
        """Parse schema content from a string."""
        schema = SchemaDef(source_file=source_file)
        current_message: Optional[MessageDef] = None
        pending_comment: Optional[str] = None
        
        for line_num, line in enumerate(content.split("\n"), 1):
            # Remove trailing whitespace
            line = line.rstrip()
            
            # Skip empty lines
            if not line.strip():
                pending_comment = None
                continue
            
            # Check for standalone comment
            stripped = line.strip()
            if stripped.startswith("#"):
                pending_comment = stripped[1:].strip()
                continue
            
            # Check for package declaration
            match = self.PACKAGE_PATTERN.match(line)
            if match:
                schema.package = match.group(1)
                pending_comment = None
                continue
            
            # Check for import
            match = self.IMPORT_PATTERN.match(line)
            if match:
                schema.imports.append(match.group(2))
                pending_comment = None
                continue
            
            # Check for message declaration
            match = self.MESSAGE_PATTERN.match(line)
            if match:
                msg_name = match.group(1)
                current_message = MessageDef(name=msg_name, comment=pending_comment)
                schema.messages[msg_name] = current_message
                pending_comment = None
                continue
            
            # Check for field definition (must be inside a message)
            match = self.FIELD_PATTERN.match(line)
            if match and current_message:
                type_name = match.group(1)
                array_size = int(match.group(2)) if match.group(2) else None
                field_name = match.group(3)
                comment = match.group(4) or pending_comment
                
                # Determine if this is a primitive or nested type
                if type_name.lower() in TYPE_MAP:
                    field_type = TYPE_MAP[type_name.lower()]
                    is_nested = False
                else:
                    field_type = FieldType.CUSTOM
                    is_nested = True
                
                field_def = FieldDef(
                    name=field_name,
                    type_name=type_name,
                    field_type=field_type,
                    array_size=array_size,
                    comment=comment,
                    is_nested=is_nested,
                )
                current_message.fields.append(field_def)
                pending_comment = None
                continue
            
            # Unknown line format
            if line.strip():
                print(f"Warning: Unrecognized line {line_num}: {line}")
        
        # Validate nested types
        self._validate_nested_types(schema)
        
        return schema
    
    def _validate_nested_types(self, schema: SchemaDef) -> None:
        """Validate that all nested types are defined."""
        for msg in schema.messages.values():
            for field_def in msg.fields:
                if field_def.is_nested:
                    if field_def.type_name not in schema.messages:
                        raise ValueError(
                            f"Unknown type '{field_def.type_name}' in message '{msg.name}'"
                        )
    
    def get_dependency_order(self, schema: SchemaDef) -> List[str]:
        """
        Get message names in dependency order (dependencies first).
        
        This ensures that when generating code, nested types are defined
        before the types that use them.
        """
        visited = set()
        order = []
        
        def visit(msg_name: str) -> None:
            if msg_name in visited:
                return
            visited.add(msg_name)
            
            msg = schema.messages.get(msg_name)
            if msg:
                for field_def in msg.fields:
                    if field_def.is_nested:
                        visit(field_def.type_name)
                order.append(msg_name)
        
        for msg_name in schema.messages:
            visit(msg_name)
        
        return order
