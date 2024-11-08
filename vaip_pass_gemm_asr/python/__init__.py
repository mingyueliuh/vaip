# -------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation.  All rights reserved.
# Licensed under the MIT License.  See License.txt in the project root for
# license information.
# Copyright (C) 2023 Advanced Micro Devices, Inc. All rights reserved.
# --------------------------------------------------------------------------
"""
JIT interface implementing packed functions that
import and compile frontend models
"""
from .ort import onnx_compile  # noqa: F401
