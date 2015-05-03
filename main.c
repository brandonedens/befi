/** @file
 * Implementation of main entry for the interpreter.
 *
 *==============================================================================
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 *==============================================================================
 *
 * @author  Brandon Edens
 * @date    2015-04-21
 * @details
 *
 */

/*******************************************************************************
 * Include Files
 */
#include <assert.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "befi.tab.h"

/*******************************************************************************
 * Constants
 */

/*******************************************************************************
 * Local Types
 */

/*******************************************************************************
 * Macros
 */
#define die(...)                                                               \
	do {                                                                   \
		fprintf(stderr, "%s:%d %s(): %s\n", __FILE__, __LINE__,        \
		        __func__, __VA_ARGS__);                                \
		exit(EXIT_FAILURE);                                            \
	} while (0)

/*******************************************************************************
 * Local Variables
 */

bool shutdown;

/*******************************************************************************
 * Global Variable Definitions
 */

/*******************************************************************************
 * Local Functions
 */

/******************************************************************************/

int main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	while (!shutdown) {
		printf("befi $ ");
		yyparse();
		printf("\n");
	}

	return EXIT_SUCCESS;
}
