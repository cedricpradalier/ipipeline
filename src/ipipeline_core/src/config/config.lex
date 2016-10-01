%{
#include "config.tab.hpp"

size_t configCurrentLine;
//#define PRINT(s) fprintf(stderr,s)
#define PRINT(s) 
%}

%option noyywrap

DIGIT 	[0-9]
NATURAL [0-9]+
ID 		[a-zA-Z_][a-zA-Z0-9_]*

%%

[-+]?{NATURAL}	PRINT("TOK_INTEGER ");return TOK_INTEGER;

[-+]?{NATURAL}("."{NATURAL})?([eE][-+]?{NATURAL})?	PRINT("TOK_FLOAT ");return TOK_FLOAT;


[tT][rR][uU][eE]		PRINT("TOK_TRUE ");return TOK_TRUE;
[fF][aA][lL][sS][eE]	PRINT("TOK_FALSE ");return TOK_FALSE;

"="					PRINT("TOK_EQ ");return TOK_EQ;
"$"					PRINT("TOK_DOLLAR ");return TOK_DOLLAR;

"\""				PRINT("TOK_DQUOTE ");return TOK_DQUOTE;

"\\".				PRINT("TOK_ESCCHAR ");return TOK_ESCCHAR;

"["					PRINT("TOK_BRACKET_OPEN ");return TOK_BRACKET_OPEN;
"]"					PRINT("TOK_BRACKET_CLOSE ");return TOK_BRACKET_CLOSE;

"{"					PRINT("TOK_BRACE_OPEN ");return TOK_BRACE_OPEN;
"}"					PRINT("TOK_BRACE_CLOSE ");return TOK_BRACE_CLOSE;
"."					PRINT("TOK_DOT ");return TOK_DOT;

"#"[^\n]*	/* Commented line */			

[ \t\r]+    PRINT("TOK_WSPACE ");return TOK_WSPACE;

\n			configCurrentLine += 1;PRINT("TOK_NEWLINE ");return TOK_NEWLINE;

{ID}		PRINT("TOK_ID ");return TOK_ID;

.			PRINT("TOK_CHAR ");return TOK_CHAR;

%%




