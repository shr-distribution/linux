/*
 * Copyright (c) 2015-2018 TrustKernel Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef TEE_CLIENT_API_H
#define TEE_CLIENT_API_H

#include "tee_kernel_api.h"

#define teec_uuid TEEC_UUID
#define teec_context TEEC_Context
#define teec_session TEEC_Session
#define teec_shared_memory TEEC_SharedMemory
#define teec_temp_memory_reference TEEC_TempMemoryReference
#define teec_registered_memory_reference TEEC_RegisteredMemoryReference
#define teec_value TEEC_Value
#define teec_parameter TEEC_Parameter
#define teec_operation TEEC_Operation

/*
 * FUNCTIONS
 */
#define teec_initialize_context TEEC_InitializeContext
#define teec_finalize_context TEEC_FinalizeContext
#define teec_register_shared_memory TEEC_RegisterSharedMemory
#define teec_allocate_shared_memory TEEC_AllocateSharedMemory
#define teec_release_shared_memory TEEC_ReleaseSharedMemory
#define teec_open_session TEEC_OpenSession
#define teec_close_session TEEC_CloseSession
#define teec_invoke_command TEEC_InvokeCommand
#define teec_request_cancellation TEEC_RequestCancellation

#endif
