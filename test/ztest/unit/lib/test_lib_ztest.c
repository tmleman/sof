// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation. All rights reserved.
//
// Author: Converted from CMock to Ztest

#include <zephyr/ztest.h>
#include <rtos/string.h>

/**
 * @brief Test rstrlen functionality for empty string
 *
 * Tests that rstrlen returns 0 for an empty string
 */
ZTEST(sof_lib_suite, test_rstrlen_for_empty_str_equals_0)
{
	int r;
	const char *str = "";

	r = rstrlen(str);
	zassert_equal(r, 0, "rstrlen should return 0 for empty string");
}

/**
 * @brief Test rstrlen functionality for null string
 *
 * Tests that rstrlen returns 0 for a null string
 */
ZTEST(sof_lib_suite, test_rstrlen_for_null_str_equals_0)
{
	int r;
	const char *str = "\0";

	r = rstrlen(str);
	zassert_equal(r, 0, "rstrlen should return 0 for null string");
}

/**
 * @brief Test rstrlen functionality for single character
 *
 * Tests that rstrlen returns 1 for a single character string
 */
ZTEST(sof_lib_suite, test_rstrlen_for_a_equals_1)
{
	int r;
	const char *str = "a";

	r = rstrlen(str);
	zassert_equal(r, 1, "rstrlen should return 1 for single character string");
}

/**
 * @brief Test rstrlen functionality for multiple characters
 *
 * Tests that rstrlen returns 4 for a string with 4 characters
 */
ZTEST(sof_lib_suite, test_rstrlen_for_abcd_equals_4)
{
	int r;
	const char *str = "abcd";

	r = rstrlen(str);
	zassert_equal(r, 4, "rstrlen should return 4 for string 'abcd'");
}

/**
 * @brief Test rstrlen functionality for a very long string
 *
 * Tests that rstrlen returns the correct length for a very long string
 */
ZTEST(sof_lib_suite, test_rstrlen_for_verylongstr)
{
	int r;
	int i;
	const int size = 2048;
	char str[size + 1];

	for (i = 0; i < size; ++i)
		str[i] = 'a';
	str[size] = 0;

	r = rstrlen(str);
	zassert_equal(r, size, "rstrlen should return correct size for very long string");
}

/**
 * @brief Test rstrlen functionality for strings with multiple nulls
 *
 * Tests that rstrlen returns the length up to the first null character
 */
ZTEST(sof_lib_suite, test_rstrlen_for_multiple_nulls_equals_5)
{
	int r;
	const char *str = "Lorem\0Ipsum\0";

	r = rstrlen(str);
	zassert_equal(r, 5, "rstrlen should return length up to first null character");
}

/**
 * @brief Test rstrcmp functionality for identical strings
 *
 * Tests that rstrcmp returns 0 for identical strings
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_a_and_a_equals_0)
{
	int r;
	const char *str1 = "a";
	const char *str2 = "a";

	r = rstrcmp(str1, str2);
	zassert_equal(r, 0, "rstrcmp should return 0 for identical strings");
}

/**
 * @brief Test rstrcmp functionality for first string < second string
 *
 * Tests that rstrcmp returns a negative value when first string is
 * lexicographically less than second string
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_a_and_b_is_negative)
{
	int r;
	const char *str1 = "a";
	const char *str2 = "b";

	r = rstrcmp(str1, str2);
	zassert_true(r < 0, "rstrcmp should return negative value when str1 < str2");
}

/**
 * @brief Test rstrcmp functionality for first string > second string
 *
 * Tests that rstrcmp returns a positive value when first string is
 * lexicographically greater than second string
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_b_and_a_is_positive)
{
	int r;
	const char *str1 = "b";
	const char *str2 = "a";

	r = rstrcmp(str1, str2);
	zassert_true(r > 0, "rstrcmp should return positive value when str1 > str2");
}

/**
 * @brief Test rstrcmp functionality for empty strings
 *
 * Tests that rstrcmp returns 0 for empty and null strings
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_empty_and_null_str_equals_0)
{
	int r;
	const char *str1 = "";
	const char *str2 = "\0";

	r = rstrcmp(str1, str2);
	zassert_equal(r, 0, "rstrcmp should return 0 for empty and null strings");
}

/**
 * @brief Test rstrcmp functionality for different length strings
 *
 * Tests that rstrcmp returns a negative value when first string is shorter
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_abc_and_abcd_is_negative)
{
	int r;
	const char *str1 = "abc";
	const char *str2 = "abcd";

	r = rstrcmp(str1, str2);
	zassert_true(r < 0, "rstrcmp should return negative when str1 is shorter");
}

/**
 * @brief Test rstrcmp functionality for different length strings
 *
 * Tests that rstrcmp returns a positive value when first string is longer
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_abcd_and_abc_is_positive)
{
	int r;
	const char *str1 = "abcd";
	const char *str2 = "abc";

	r = rstrcmp(str1, str2);
	zassert_true(r > 0, "rstrcmp should return positive when str1 is longer");
}

/**
 * @brief Test rstrcmp functionality for case sensitivity
 *
 * Tests that rstrcmp is case sensitive
 */
ZTEST(sof_lib_suite, test_rstrcmp_for_abc_and_abc_upper_b_is_positive)
{
	int r;
	const char *str1 = "abc";
	const char *str2 = "aBc";

	r = rstrcmp(str1, str2);
	zassert_true(r > 0, "rstrcmp should be case sensitive");
}

/**
 * @brief Define and initialize the test suite
 */
ZTEST_SUITE(sof_lib_suite, NULL, NULL, NULL, NULL, NULL);
