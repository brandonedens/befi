/* Brandon Edens Flight Interpreter Syntax definition.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 */

%{
#include <stdlib.h>
#include <stdio.h>

#include "befi.tab.h"

extern YYSTYPE yylval;

void yyerror(char *s)
{
	printf("line %d: %s at \"%s\"\n", yylineno, s, yytext);
}

%}

%option noyywrap nodefault yylineno case-insensitive

%%
alt                     return TOKEN_ALT;
coord                   return TOKEN_COORD;
exec                    return TOKEN_EXEC;
exit                    return TOKEN_EXIT;
flyto                   return TOKEN_FLYTO;
land                    return TOKEN_LAND;
loiter                  return TOKEN_LOITER;
remove                  return TOKEN_REMOVE;
route                   return TOKEN_ROUTE;
set                     return TOKEN_SET;
sleep                   return TOKEN_SLEEP;
takeoff                 return TOKEN_TAKEOFF;
waypoint                return TOKEN_WAYPOINT;

-?[0-9]+\.[0-9]+        yylval.decimal = strtof(yytext, NULL); return DECIMAL;
-?[A-Za-z][A-Za-z0-9_]* yylval.string = strdup(yytext); return IDENTIFIER;
[0-9]+                  yylval.number = atoi(yytext); return NUMBER;


\n                      /* ignore end of line */;
[ \t]+                  /* ignore whitespace */;
.                       return (int) yytext[0];
%%
