%token TOK_STRING TOK_TRUE TOK_FALSE TOK_EQ TOK_DOLLAR
%token TOK_ID TOK_NEWLINE TOK_DQUOTE TOK_ESCCHAR TOK_CHAR
%token TOK_FLOAT TOK_INTEGER TOK_WSPACE TOK_DOT
%token TOK_BRACKET_OPEN TOK_BRACKET_CLOSE TOK_BRACE_OPEN TOK_BRACE_CLOSE


%start Configuration

%{
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include "ipipeline_core/Config.h"


//extern "C" {

extern size_t configCurrentLine;
extern char * configtext;
int configlex();
static void configerror(const char *s)
{
	fprintf( stderr, "line %6d : %s\n", configCurrentLine,s );
}

static void semanticError(const char * s)
{
	fprintf( stderr, "line %6d : %s\n", configCurrentLine,s );
}


static Config* cfg;
static std::string currentID,currentSection;
static std::string dollaredSection;
static std::string dollaredID;
static std::string stringValue;
static bool is_cfg,is_env;
bool invalidTypesEncountered;
bool inexistantIdEncountered;

#define YYPARSE_PARAM vcfg

%}


%%

Configuration : {
			  		configCurrentLine = 1;
					invalidTypesEncountered =
				    inexistantIdEncountered = false;
					cfg = ((Config*)vcfg);
					cfg->selectSection("default",true);
				}
			  ConfigLine ConfigList 
			  ;

ConfigList : ConfigLine ConfigList | ;

Blank : Blank TOK_WSPACE | ;

ConfigLine : Blank TOK_ID 
		{
			currentID.assign(configtext);
		}
		Blank TOK_EQ Blank
		Value 
		Blank TOK_NEWLINE
	|
		Blank TOK_BRACKET_OPEN Blank 
		SectionId 
		{
			size_t begin, end;
			begin = currentSection.find_first_not_of(" \n\r\t");
			end = currentSection.find_last_not_of(" \n\r\t");
			currentSection = currentSection.substr(begin,end-begin+1);
			cfg->selectSection(currentSection,true);
		}
		Blank TOK_BRACKET_CLOSE Blank TOK_NEWLINE
	|
		Blank TOK_NEWLINE
	|
		error TOK_NEWLINE { yyerrok; }
	;

SectionWord : TOK_INTEGER | TOK_FLOAT | TOK_TRUE | TOK_FALSE | TOK_WSPACE | TOK_ID | TOK_CHAR | TOK_DOT;

EndOfSectionId : | EndOfSectionId SectionWord { currentSection += configtext; } ;

SectionId : TOK_ID {currentSection.assign(configtext);} 
		  EndOfSectionId ;

EndOfQualifiedID : TOK_DOT TOK_ID 
	{
		dollaredSection.assign(dollaredID);
		dollaredID.assign(configtext);
	}
	| ;

QualifiedID : TOK_ID 
	{
		dollaredSection.clear();
		dollaredID.assign(configtext);
	}
	EndOfQualifiedID;

DollaredID : TOK_DOLLAR QualifiedID
	{
		is_cfg = cfg->exists(dollaredID,dollaredSection);
		is_env = (getenv(dollaredID.c_str()) != NULL);
		// printf("Get Env '%s' -> %s\n",dollaredID.c_str(),getenv(dollaredID.c_str()));
		if (!is_env && !is_cfg) {
			char msg[128];
			sprintf(msg,"Inexistant ID '$%s%s%s'", dollaredSection.c_str(),
				(dollaredSection.empty())?"":".", dollaredID.c_str());
			semanticError(msg);
			inexistantIdEncountered = true;
			dollaredID.clear();
			dollaredSection.clear();
		}
	}
	| TOK_DOLLAR TOK_BRACE_OPEN QualifiedID 
	{
		is_cfg = cfg->exists(dollaredID,dollaredSection);
		is_env = (getenv(dollaredID.c_str()) != NULL);
		if (!is_env && !is_cfg) {
			char msg[128];
			sprintf(msg,"Inexistant ID '$%s%s%s'", dollaredSection.c_str(),
				(dollaredSection.empty())?"":".", dollaredID.c_str());
			semanticError(msg);
			inexistantIdEncountered = true;
			dollaredID.clear();
			dollaredSection.clear();
		}
	} TOK_BRACE_CLOSE 
	;

Word1 : TOK_INTEGER | TOK_FLOAT | TOK_TRUE | TOK_FALSE | 
	  TOK_EQ | TOK_WSPACE | TOK_ID | TOK_CHAR | TOK_DOT |
	  TOK_BRACKET_OPEN | TOK_BRACKET_CLOSE |
	  TOK_BRACE_OPEN | TOK_BRACE_CLOSE ;
		  
Word : Word1
	{
		stringValue += configtext;
	}
	| DollaredID
	{
		if (is_cfg) {
			std::string value;
			if (dollaredSection.empty()) {
				if (cfg->getString(dollaredID,value)) {
					stringValue += value;
				} else {
					stringValue += "$";
					stringValue += dollaredID;
				}
			} else {
				cfg->pushSection();
				cfg->selectSection(dollaredSection);
				if (cfg->getString(dollaredID,value)) {
					stringValue += value;
				} else {
					stringValue += "$";
					stringValue += dollaredSection;
					stringValue += ".";
					stringValue += dollaredID;
				}
				cfg->popSection();
			}
		} else if (is_env) {
			stringValue += getenv(dollaredID.c_str());
		}
	}
	| TOK_ESCCHAR
	{
		switch (configtext[1]) {
			case '"' : 
				stringValue += '"'; break;
			case 'r' : 
				stringValue += '\r'; break;
			case 'n' : 
				stringValue += '\n'; break;
			case 't' : 
				stringValue += '\t'; break;
			case '$' : 
				stringValue += '$'; break;
			default : 
				stringValue += configtext; break;
		}
	}
	;

CharList : | CharList Word;

String : TOK_DQUOTE
	{
		stringValue.clear();
	}
	CharList 
	TOK_DQUOTE
	;


Value : DollaredID
	{
		bool r;
		if (!dollaredID.empty()) {
			r = cfg->affectID(currentID,dollaredSection,dollaredID);
			if (!r) 
			{
				char msg[128];
				if (dollaredSection.empty()) {
					sprintf(msg,"Invalid id value ${%s.%s} for id '%s'",
						cfg->currentSection().c_str(),dollaredID.c_str(),
						currentID.c_str());
				} else {
					sprintf(msg,"Invalid id value ${%s.%s} for id '%s'",
						dollaredSection.c_str(),dollaredID.c_str(),
						currentID.c_str());
				}
				semanticError(msg);
				invalidTypesEncountered = true;
			}

		}
	}
	| TOK_INTEGER 
	{	
		int i;bool r;
		sscanf(configtext,"%d",&(i));
		r = cfg->affectInt(currentID,i);
		if (!r) 
		{
			char msg[128];
			sprintf(msg,"Invalid int value '%s' for id '%s.%s'",
				configtext,cfg->currentSection().c_str(),currentID.c_str());
			semanticError(msg);
			invalidTypesEncountered = true;
		}
	}
	| TOK_FLOAT		
	{	
		double d;bool r;
		sscanf(configtext,"%le",&(d));
		r = cfg->affectDouble(currentID,d);
		if (!r) 
		{
			char msg[128];
			sprintf(msg,"Invalid float value '%s' for id '%s.%s''",
				configtext,cfg->currentSection().c_str(),currentID.c_str());
			semanticError(msg);
			invalidTypesEncountered = true;
		}
	}
	| BoolValue
	| String	
	{
		bool r;
		r = cfg->affectString(currentID.c_str(),stringValue);
		if (!r) 
		{
			char msg[128];
			sprintf(msg,"Invalid string value '%s' for id '%s.%s'",
				stringValue.c_str(),cfg->currentSection().c_str(),currentID.c_str());
			semanticError(msg);
			invalidTypesEncountered = true;
		}
	}
	;

BoolValue : TOK_TRUE 	
	{	
		bool r;
		r = cfg->affectBool(currentID,true);
		if (!r) 
		{
			char msg[128];
			sprintf(msg,"Invalid bool value '%s' for id '%s.%s'",
				configtext,cfg->currentSection().c_str(),currentID.c_str());
			semanticError(msg);
			invalidTypesEncountered = true;
		}
	}
	| TOK_FALSE			
	{	
		bool r;
		r = cfg->affectBool(currentID,false);
		if (!r) 
		{
			char msg[128];
			sprintf(msg,"Invalid bool value '%s' for id '%s.%s'",
				configtext,cfg->currentSection().c_str(),currentID.c_str());
			semanticError(msg);
			invalidTypesEncountered = true;
		}
	}
	;


%%
//}

