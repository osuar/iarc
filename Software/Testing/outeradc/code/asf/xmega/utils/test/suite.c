/**
 * \file
 *
 * \brief Test suite core functionality
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
 * 4. This software may only be redistributed and used in connection with an
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
#include <compiler.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <test/suite.h>	

/**
 * \weakgroup test_suite_group
 * @{
 */

//! \internal \see test_set_data(), test_get_data()
void *test_priv_data;

/**
 * \internal
 * Context saved before executing a test or fixture function. Used for
 * doing non-local jumps from the test cases on failure.
 */
static jmp_buf	test_failure_jmpbuf;

static void test_report_failure(const struct test_case *test,
		const char *stage, int result)
{
	dbg_error("Test '%s' failed during '%s': %d\n",
			test->name, stage, result);
}

/**
 * \internal
 * Call a test or fixture function
 *
 * This function will initialize test_failure_jmpbuf and call \a func
 * with \a test as the parameter.
 *
 * \return #TEST_PASS if \a func was executed successfully, or the
 * result value passed to test_fail() on failure.
 */
static int test_call(void (*func)(const struct test_case *), 
	const struct test_case *test)
{
	int	ret = 0;

	if (!func)
		return TEST_PASS;

	/*
	 * The first call to setjmp() always return 0. However, if the
	 * call to func() below goes wrong, we'll return here again with
	 * a nonzero value.
	 */
	ret = setjmp(test_failure_jmpbuf);
	if (ret)
		return ret;

	func(test);

	return TEST_PASS;
}

static int test_case_run(const struct test_case *test)
{
	int	result;

	dbg_info("Running test: %s\n", test->name);
	if (test->setup) {
		int	ret;
		dbg("Setting up fixture\n");
		ret = test_call(test->setup, test);
		if (ret) {
			test_report_failure(test, "setup", ret);
			result = ret;
			goto out;
		}
	}

	result = test_call(test->run, test);
	if (result)
		test_report_failure(test, "test", result);

	if (test->cleanup) {
		int	ret;
		dbg("Cleaning up fixture\n");
		ret = test_call(test->cleanup, test);
		if (ret && !result) {
			test_report_failure(test, "cleanup", ret);
			result = ret;
		}
	}

out:

	return result;
}

/**
 * \brief Run a test suite
 *
 * Run all tests in \a suite, in the order in which they are found in
 * the array.
 *
 * \return The number of tests that didn't pass.
 */
int test_suite_run(const struct test_suite *suite)
{
	unsigned int	nr_failures = 0;
	unsigned int	nr_errors = 0;
	unsigned int    nr_tests;
	const struct test_case *const *tests;
	unsigned int	i;
	int		ret;

	dbg_info("Running test suite '%s'...\n", suite->name);

	nr_tests = suite->nr_tests;
	tests = suite->tests;

	for (i = 0; i < nr_tests; i++) {
		const struct test_case *test;

		test = tests[i];
		ret = test_case_run(test);
		if (ret < 0)
			nr_errors++;
		else if (ret > 0)
			nr_failures++;
	}

	dbg_info("Test suite '%s' complete: %u tests, %u failures, %u errors\n\n",
			suite->name,
			nr_tests, nr_failures, nr_errors);

	return nr_errors + nr_failures;
}

//__printf_format(5, 6) __noreturn
void test_priv_fail(const struct test_case *test, int result,
		const char *file, unsigned int line,
		const char __progmem_arg *fmt, ...)
{
	va_list	ap;

	dbg_error("Test '%s' failed at %s:%u:\n\t", test->name,
			file, line);

	va_start(ap, fmt);
	dbg_vprintf_pgm(fmt, ap);
	va_end(ap);
	dbg_putchar('\n');

	/*
	 * This will cause the setjmp() call in test_call() to return
	 * TEST_FAILED.
	 */
	longjmp(test_failure_jmpbuf, result);
}

//! @}
