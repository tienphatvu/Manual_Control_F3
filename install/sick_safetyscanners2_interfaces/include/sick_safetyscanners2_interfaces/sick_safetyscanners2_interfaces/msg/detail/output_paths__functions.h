// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sick_safetyscanners2_interfaces:msg/OutputPaths.idl
// generated code does not contain a copyright notice

#ifndef SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__FUNCTIONS_H_
#define SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sick_safetyscanners2_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "sick_safetyscanners2_interfaces/msg/detail/output_paths__struct.h"

/// Initialize msg/OutputPaths message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sick_safetyscanners2_interfaces__msg__OutputPaths
 * )) before or use
 * sick_safetyscanners2_interfaces__msg__OutputPaths__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__init(sick_safetyscanners2_interfaces__msg__OutputPaths * msg);

/// Finalize msg/OutputPaths message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
void
sick_safetyscanners2_interfaces__msg__OutputPaths__fini(sick_safetyscanners2_interfaces__msg__OutputPaths * msg);

/// Create msg/OutputPaths message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
sick_safetyscanners2_interfaces__msg__OutputPaths *
sick_safetyscanners2_interfaces__msg__OutputPaths__create();

/// Destroy msg/OutputPaths message.
/**
 * It calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
void
sick_safetyscanners2_interfaces__msg__OutputPaths__destroy(sick_safetyscanners2_interfaces__msg__OutputPaths * msg);

/// Check for msg/OutputPaths message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__are_equal(const sick_safetyscanners2_interfaces__msg__OutputPaths * lhs, const sick_safetyscanners2_interfaces__msg__OutputPaths * rhs);

/// Copy a msg/OutputPaths message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__copy(
  const sick_safetyscanners2_interfaces__msg__OutputPaths * input,
  sick_safetyscanners2_interfaces__msg__OutputPaths * output);

/// Initialize array of msg/OutputPaths messages.
/**
 * It allocates the memory for the number of elements and calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__init(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array, size_t size);

/// Finalize array of msg/OutputPaths messages.
/**
 * It calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
void
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__fini(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array);

/// Create array of msg/OutputPaths messages.
/**
 * It allocates the memory for the array and calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence *
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__create(size_t size);

/// Destroy array of msg/OutputPaths messages.
/**
 * It calls
 * sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
void
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__destroy(sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * array);

/// Check for msg/OutputPaths message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__are_equal(const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * lhs, const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * rhs);

/// Copy an array of msg/OutputPaths messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sick_safetyscanners2_interfaces
bool
sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence__copy(
  const sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * input,
  sick_safetyscanners2_interfaces__msg__OutputPaths__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SICK_SAFETYSCANNERS2_INTERFACES__MSG__DETAIL__OUTPUT_PATHS__FUNCTIONS_H_
