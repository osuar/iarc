/**
 * \file
 *
 * \brief Test suite core declarations
 *
 * Copyright (C) 2010 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software m ay only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
#ifndef TEST_SUITE_H_INCLUDED
#define TEST_SUITE_H_INCLUDED

#include <compiler.h>
#include <stdio.h>
#if defined(XMEGA)
#  include <progmem.h>
#endif
/**
 * \defgroup test_suite_group Test Suite Framework
 * @{
 */

/**
 * \brief Wrappers for printing debug-information
 *
 * The stream must be set up by the test-application
 * somehow, for now uses printf.
 */
#if defined(XMEGA)
#define dbg(__fmt_)						\
	printf_P(PSTR(__fmt_))
#define dbg_info(__fmt_, ...)						\
	printf_P(PSTR(__fmt_), __VA_ARGS__)
#define dbg_error(_x, ...)						\
	printf_P(PSTR(_x), __VA_ARGS__)
#define dbg_putchar(c)	\
	putc(c, stdout)
#define dbg_vprintf_pgm(...) \
	vfprintf_P(stdout, __VA_ARGS__)
#else
#define dbg(__fmt_)						\
	printf(__fmt_)
#define dbg_info(__fmt_, ...)						\
	printf(__fmt_, __VA_ARGS__)
#define dbg_error(_x, ...)						\
	printf(_x, __VA_ARGS__)
#define dbg_putchar(c)	\
	putc(c, stdout)
#define dbg_vprintf_pgm(...) \
	vfprintf(stdout, __VA_ARGS__)
#endif
	
/**
 * \brief Progmem-related defines.
 *
 * Not relevant until printf is implemented directly in asf and supports
 * arguments stored in progmem.
 */
#define __progmem_arg 
 
 
/**
 * \brief Status codes returned by test cases and fixtures
 *
 * Note that test cases and especially fixtures may return any of the
 * status codes defined by #status_code as well.
 */
enum test_status {
	TEST_PASS		= 0,		//!< Test succeeded
	TEST_FAILED		= 1,		//!< Test failed
};

/**
 * \brief A single test case
 *
 * This structure represents a test case which tests one specific
 * feature or behavior.
 */
struct test_case {
	/**
	 * \brief Set up the environment in which \a test is to be run
	 *
	 * This may involve allocating memory, initializing hardware,
	 * etc. If something goes wrong, this function may call
	 * test_fail(), normally with a negative status code.
	 */
	void (*setup)(const struct test_case *test);
	//! Run the test
	void (*run)(const struct test_case *test);
	/**
	 * \brief Clean up the environment after \a test has been run
	 *
	 * This may freeing any memory allocated by setup(), gracefully
	 * shutting down hardware, etc. If something goes wrong, this
	 * function may call test_fail(), normally with a negative
	 * status code.
	 */
	void (*cleanup)(const struct test_case *test);
	//! The name of this test
	const char *name;
};

/**
 * \brief Convenience macro for creating a test case struct.
 * \param _sym Variable name of the resulting struct
 * \param _setup Function which sets up a test case environment. Can be NULL.
 * \param _run Test function
 * \param _cleanup Function which cleans up what was set up. Can be NULL.
 * \param _name String describing the test case.
 */
#define DEFINE_TEST_CASE(_sym, _setup, _run, _cleanup, _name) \
	static const char _test_str_##_sym[] = _name;             \
	static const struct test_case _sym = {                    \
		.setup			= _setup,						      \
		.run            = _run,                               \
		.cleanup        = _cleanup,                           \
		.name           = _test_str_##_sym                    \
	}

/**
 * \brief Convenience macro for creating an array of test cases.
 * @param _sym Variable name of the resulting array
 */
#define DEFINE_TEST_ARRAY(_sym)                          \
	const struct test_case *const _sym[]


/**
 * \brief A test suite
 *
 * A test suite may contain one or more test cases which are to be run
 * in sequence.
 */
struct test_suite {
	//! The number of tests in this suite
	unsigned int		        nr_tests;
	//! Array of pointers to the test cases
	const struct test_case *const *tests;
	//! The name of the test suite
	const char        *name;
};
#ifndef ARRAY_LEN
# define ARRAY_LEN(a)	(sizeof(a) / sizeof((a)[0]))
#endif

/**
 * \brief Convenience macro for creating a test suite.
 * \param _sym Variable name of the resulting struct
 * \param _test_array Array of test cases, created with DEFINE_TEST_ARRAY()
 * \param _name String describing the test suite.
 */
#define DEFINE_TEST_SUITE(_sym, _test_array, _name)                     \
	static const char _test_str_##_sym[] = _name;        \
	const struct test_suite _sym = {                     \
		.nr_tests       = ARRAY_LEN(_test_array),               \
		.tests          = _test_array,                          \
		.name           = _test_str_##_sym                     \
	}
#define DECLARE_TEST_SUITE(_sym)                                        \
	const struct test_suite _sym

extern int test_suite_run(const struct test_suite *suite);

extern void *test_priv_data;

/**
 * \brief Set private data pointer for the current test.
 *
 * \param data Pointer to arbitrary run-time data for the test currently
 * being run.
 */
static inline void test_set_data(void *data)
{
	test_priv_data = data;
}

/**
 * \brief Get the private data pointer for the current test.
 *
 * \return Pointer to arbitrary run-time data for the test currently
 * being run, previously registered using test_set_data().
 */
static inline void *test_get_data(void)
{
	return test_priv_data;
}

//strcpy(test_priv_file[] = __FILE__; // Usable for implementations of printf
                                      // which accept arguments from progmem.

//__printf_format(5, 6) __noreturn
extern void test_priv_fail(const struct test_case *test,
		int result, const char *file, unsigned int line,
		const char __progmem_arg *fmt, ...);

/**
 * \def test_fail(test, result, ...)
 * \brief Fail the test
 *
 * Calling this function will cause the test to terminate with a
 * failure. It will return directly to the test suite core, not to the
 * caller.
 *
 * \param test Test case which failed
 * \param result The result of the test (may not be 0)
 * \param ... printf()-style format string and arguments
 */

#if defined(CONFIG_PROGMEM)
#  define test_priv_fail_ps(test, result, format, ...)                    \
		do {                                                            \
			static PROGMEM_DECLARE(char, _fmtstr[]) = format "%s";   \
			test_priv_fail(test, result, __FILE__, __LINE__,  \
					_fmtstr, __VA_ARGS__);                  \
		} while (0)

#  define test_fail(test, result, ...)					\
	test_priv_fail_ps(test, result, __VA_ARGS__, "")
#else
#  define test_fail(test, result, ...)					\
		test_priv_fail(test, result, __FILE__, __LINE__,          \
				__VA_ARGS__)
#endif

/**
 * \brief Verify that \a condition is true
 *
 * If \a condition is false, fail the test with an error message
 * indicating the condition which failed.
 *
 * \param test The test case currently being run
 * \param condition Expression to be validated
 * \param ... Format string and arguments
 */
#define test_fail_unless(test, condition, ...)                          \
	do {                                                            \
		if (!(condition))                                       \
			test_fail(test, TEST_FAILED, __VA_ARGS__);      \
	} while (0)

//! @}

#endif /* TEST_SUITE_H_INCLUDED */
