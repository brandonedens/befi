/* Brandon Edens Flight Interpreter Syntax definition.
 * 2015-04-22
 * Copyright 2015 by Brandon Edens. All Rights Reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
