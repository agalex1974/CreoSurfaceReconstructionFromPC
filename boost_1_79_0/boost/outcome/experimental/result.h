/* C interface for result
(C) 2017-2022 Niall Douglas <http://www.nedproductions.biz/> (6 commits)
File Created: Aug 2017


Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef BOOST_OUTCOME_EXPERIMENTAL_RESULT_H
#define BOOST_OUTCOME_EXPERIMENTAL_RESULT_H

#include <stdint.h>  // for intptr_t

#ifdef __cplusplus
extern "C"
{
#endif

#define BOOST_OUTCOME_C_DECLARE_RESULT(ident, R, S)                                                                                                                        \
  struct cxx_result_##ident                                                                                                                                    \
  {                                                                                                                                                            \
    union                                                                                                                                                      \
    {                                                                                                                                                          \
      R value;                                                                                                                                                 \
      S error;                                                                                                                                                 \
    };                                                                                                                                                         \
    unsigned flags;                                                                                                                                            \
  }

#define BOOST_OUTCOME_C_RESULT(ident) struct cxx_result_##ident


#define BOOST_OUTCOME_C_RESULT_HAS_VALUE(r) (((r).flags & 1U) == 1U)

#define BOOST_OUTCOME_C_RESULT_HAS_ERROR(r) (((r).flags & 2U) == 2U)

#define BOOST_OUTCOME_C_RESULT_ERROR_IS_ERRNO(r) (((r).flags & (1U << 4U)) == (1U << 4U))


  /***************************** <system_error2> support ******************************/

#define BOOST_OUTCOME_C_DECLARE_STATUS_CODE(ident, value_type)                                                                                                             \
  struct cxx_status_code_##ident                                                                                                                               \
  {                                                                                                                                                            \
    void *domain;                                                                                                                                              \
    value_type value;                                                                                                                                          \
  };

#define BOOST_OUTCOME_C_STATUS_CODE(ident) struct cxx_status_code_##ident

#define BOOST_OUTCOME_C_DECLARE_RESULT_STATUS_CODE(ident, R, S)                                                                                                            \
  struct cxx_result_status_code_##ident                                                                                                                        \
  {                                                                                                                                                            \
    R value;                                                                                                                                                   \
    unsigned flags;                                                                                                                                            \
    S error;                                                                                                                                                   \
  }

#define BOOST_OUTCOME_C_RESULT_STATUS_CODE(ident) struct cxx_result_status_code_##ident


  struct cxx_status_code_posix
  {
    void *domain;
    int value;
  };
#define BOOST_OUTCOME_C_DECLARE_RESULT_ERRNO(ident, R) BOOST_OUTCOME_C_DECLARE_RESULT_STATUS_CODE(posix_##ident, R, struct cxx_status_code_posix)
#define BOOST_OUTCOME_C_RESULT_ERRNO(ident) BOOST_OUTCOME_C_RESULT_STATUS_CODE(posix_##ident)

  struct cxx_status_code_system
  {
    void *domain;
    intptr_t value;
  };
#define BOOST_OUTCOME_C_DECLARE_RESULT_SYSTEM(ident, R) BOOST_OUTCOME_C_DECLARE_RESULT_STATUS_CODE(system_##ident, R, struct cxx_status_code_system)
#define BOOST_OUTCOME_C_RESULT_SYSTEM(ident) BOOST_OUTCOME_C_RESULT_STATUS_CODE(system_##ident)

#ifdef __cplusplus
}
#endif

#endif
