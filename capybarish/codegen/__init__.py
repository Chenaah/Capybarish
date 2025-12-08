"""
Capybarish Code Generation Module.

This module provides LCM-style code generation from schema definitions,
supporting both Python and C++ output for cross-platform communication.

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0.
"""

from .parser import SchemaParser, MessageDef, FieldDef
from .python_gen import PythonGenerator
from .cpp_gen import CppGenerator
from .generator import CodeGenerator

__all__ = [
    "SchemaParser",
    "MessageDef",
    "FieldDef",
    "PythonGenerator",
    "CppGenerator",
    "CodeGenerator",
]
