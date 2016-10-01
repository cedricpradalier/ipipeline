/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "ipipeline_core/Config.h"

//extern "C" {
extern FILE * configin;
extern unsigned int configCurrentLine;
extern bool inexistantIdEncountered;
extern bool invalidTypesEncountered;
extern int configparse(void * vcfg);
//}

using namespace std;

#define S2C(s) (s.empty()?NULL:s.c_str())

Config::Config()
{
	ignoreInvalidTypes = ignoreInexistantID = false;
	selectSection("default",true);
}

Config::~Config()
{
	clear();
}

void Config::pushSection() 
{
	pushed_section.push_front(currentTable);
}

bool Config::popSection() 
{
	if (pushed_section.empty()) return false;
	currentTable = pushed_section.front();
	pushed_section.pop_front();
	return true;
}

bool Config::selectSection(const string & section_name,bool create)
{
	Table::iterator it = cfg.find(section_name);
	if (it == cfg.end()) {
		if (!create) return false;

		SectionItem item;
		item.name = section_name;
		item.id = sectiondata.size();
		sectiondata.push_back(item);
		pair<Table::iterator,bool> res = 
			cfg.insert(pair<string,unsigned int>(item.name,item.id));
		assert(res.second = true);
		currentTable = item.id;
		return true;
	}
	else
	{
		currentTable = it->second;
		return true;
	}
}

int Config::getIdx(const string & name,bool create)
{
	int i = getIdx(name);
	if ((i<0) && create)
		return addDefinition(name,TFREE);
	return i;
}

int Config::getIdx(const string & name) const
{
	if (name.empty()) return -1;
	const Table & T = sectiondata[currentTable].section;
	Table::const_iterator it = T.find(name);
	if (it == T.end()) {
		return -1;
	}
	else
		return (*it).second;
}


bool Config::affectEnv(const string & name,const string & id_src,bool create)
{
	const char * env = getenv(id_src.c_str());
	if (env == NULL) return false;

	int i = getIdx(name,create);
	if (i<0) return false;
	
	switch (data[i].type)
	{
		case TINT : 
			{
				int tmp;
				if (sscanf(env," %d ",&tmp) != 1) return false;
				return affectInt(i,tmp);
			}
		case TDOUBLE :
			{
				double tmp;
				if (sscanf(env," %le ",&tmp) != 1) return false;
				return affectDouble(i,tmp);
			}
		case TBOOL :
			{
				int tmp;
				if (strcasecmp(env,"true") == 0)
					return affectBool(i,true);
				if (strcasecmp(env,"false") == 0)
					return affectBool(i,false);
				if (sscanf(env," %d ",&tmp) != 1) 
					return false;
				return affectBool(i,(tmp!=0));
			}
		case TSTRING :
			{
				return affectString(i,env);
			}
		case TFREE :
			{
				int j;double d;
				int sj,sd;
				sj = sscanf(env," %d ",&j);
				sd = sscanf(env," %le ",&d);
				if (sj == 1) {
					if ((double)j == d) 
						return affectInt(i,j);
					else 
						return affectDouble(i,d);
				} else if (strcasecmp(env,"true")==0) {
					return affectBool(i,true);
				} else if (strcasecmp(env,"false")==0) {
					return affectBool(i,false);
				} else {
					return affectString(i,env);
				}
			}
		default : break;
	}
	return false;
}
	


bool Config::affectID(const string & name,const string & id_section,const string & id_name,bool create)
{
	if (id_name.empty()) return false;
	int i = -1,j=-1;

	if (!id_section.empty()) {
		pushSection();
		if (selectSection(id_section))
			j = getIdx(id_name,false);
		popSection();
	} else {
		j = getIdx(id_name,false);
	}
	if (j < 0) return affectEnv(name,id_name,create);

	i = getIdx(name,create);
	if (i<0) return false;

	if (!data[j].affected) return false;

	switch (data[j].type)
	{
		case TINT : 
			return affectInt(i,data[j].value.intValue);
		case TDOUBLE :
			return affectDouble(i,data[j].value.dblValue);
		case TBOOL :
			return affectBool(i,data[j].value.boolValue);
		case TSTRING :
			return affectString(i,data[j].value.strValue);
		default : break;
	}
	return false;
}

bool Config::affectInt(const string & name,int a,bool create) 
{
	int i = getIdx(name,create);
	if (i<0) return false;
	return affectInt(i,a);
}

bool Config::affectInt(unsigned int i,int a)
{
	switch (data[i].type)
	{
		case TFREE :
			data[i].type = TINT;
		case TINT : 
			data[i].value.intValue = a;
			data[i].affected = true;
			return true;
		case TDOUBLE :
			data[i].value.dblValue = (double)a;
			data[i].affected = true;
			return true;
		case TBOOL :
			data[i].value.boolValue = (a != 0);
			data[i].affected = true;
			return true;
		default :
			return false;
	}
	return false;
}
	
bool Config::affectDouble(const string & name,double a,bool create)
{
	int i = getIdx(name,create);
	if (i<0) return false;
	return affectDouble(i,a);
}

bool Config::affectDouble(unsigned int i,double a)
{
	if (data[i].type == TFREE) data[i].type = TDOUBLE;
	if (data[i].type != TDOUBLE) return false;
	data[i].value.dblValue = a;
	data[i].affected = true;
	return true;
}
	
bool Config::affectBool(const string & name,bool b,bool create)
{
	int i = getIdx(name,create);
	if (i<0) return false;
	return affectBool(i,b);
}

bool Config::affectBool(unsigned int i,bool b)
{
	if (data[i].type == TFREE) data[i].type = TBOOL;
	if (data[i].type != TBOOL) return false;
	data[i].value.boolValue = b;
	data[i].affected = true;
	return true;
}
	
bool Config::affectString(const string & name,const string &a,bool create) 
{
	int i = getIdx(name,create);
	if (i<0) return false;
	return affectString((unsigned int)i,a);
}

bool Config::affectString(unsigned int i,const string & a)
{
	if (data[i].type == TFREE) data[i].type = TSTRING;
	if (data[i].type != TSTRING) return false;
	if (data[i].affected)
		free(data[i].value.strValue);
	data[i].value.strValue = strdup((a.c_str()==NULL)?"":a.c_str());
	data[i].affected = true;
	return true;
}
	
unsigned int Config::addDefinition(const string & name,ConfigType t,const string & section_name)
{
	if (!section_name.empty()) {
		pushSection();
		selectSection(section_name,true);
	}
	
	ConfigItem item;
	
	item.id = data.size();
	item.affected = false;
	item.name = name;
	item.type = t;
	data.push_back(item);
	
	Table & T = sectiondata[currentTable].section;
	pair<Table::iterator,bool> res = 
		T.insert(pair<string,unsigned int>(item.name,item.id));
	assert(res.second = true);
	
	if (!section_name.empty()) {
		popSection();
	}

	return item.id;
}

bool Config::getInt(const string & name,int *  value) const
{
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TSTRING :
		case TFREE : 
			return false;
		case TBOOL : 
			*value = (data[i].value.boolValue)?1:0;
			break;
		case TINT : 
			*value = data[i].value.intValue;
			break;
		case TDOUBLE :
			*value = (int)round(data[i].value.dblValue);
			break;
	}
	return true;
}

bool Config::getUInt(const string & name,unsigned int *  value) const
{
	int val;
	if (!getInt(name,&val)) return false;
	if (val < 0) return false;
	*value = val;
	return true;
}


bool Config::getBool(const string & name,bool *  value) const
{
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TSTRING :
		case TFREE : 
			return false;
		case TBOOL : 
			*value = data[i].value.boolValue;
			break;
		case TINT : 
			*value = (data[i].value.intValue != 0);
			break;
		case TDOUBLE :
			*value = (data[i].value.dblValue != 0);
			break;
	}
	return true;
}


bool Config::getDouble(const string & name,double *  value) const
{
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TSTRING :
		case TFREE : 
		case TBOOL : 
			return false;
		case TINT : 
			*value = data[i].value.intValue;
			break;
		case TDOUBLE :
			*value = data[i].value.dblValue;
			break;
	}
	return true;
}


bool Config::getString(const string & name,string & value) const
{
	char tmp[128];
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TFREE :
			return false;
		case TSTRING :
			value = data[i].value.strValue;
			break;
		case TINT :
			snprintf(tmp,128,"%d",data[i].value.intValue);
			value = tmp;
			break;
		case TDOUBLE :
			snprintf(tmp,128,"%e",data[i].value.dblValue);
			value = tmp;
			break;
		case TBOOL :
			if (data[i].value.boolValue) 
				value = "true";
			else
				value = "false";
			break;
	}
	return true;
}

bool Config::getString(const string & name,char * value,unsigned int maxsize) const
{
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TFREE :
			return false;
		case TSTRING :
			if (strlen(data[i].value.strValue) > maxsize)
				return false;
			strcpy(value,data[i].value.strValue);
			break;
		case TINT :
			snprintf(value,maxsize,"%d",data[i].value.intValue);
			break;
		case TDOUBLE :
			snprintf(value,maxsize,"%e",data[i].value.dblValue);
			break;
		case TBOOL :
			snprintf(value,maxsize,"%s",
					data[i].value.boolValue?"true":"false");
			break;
	}
	return true;
}

bool Config::getStringDup(const string & name,char ** value) const
{
	char tmp[128];
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TFREE :
			return false;
		case TSTRING :
			*value = strdup(data[i].value.strValue);
			return true;
			break;
		case TINT :
			snprintf(tmp,128,"%d",data[i].value.intValue);
			break;
		case TDOUBLE :
			snprintf(tmp,128,"%e",data[i].value.dblValue);
			break;
		case TBOOL :
			snprintf(tmp,128,"%s",
					data[i].value.boolValue?"true":"false");
			break;
	}
	*value = strdup(tmp);
	return true;
}

bool Config::getStringLength(const string & name,unsigned int * size) const
{
	char tmp[128];
	int i = getIdx(name);
	if (i < 0) return false;
	if (! data[i].affected)
		return false;
	switch (data[i].type) {
		case TFREE :
			return false;
		case TSTRING :
			*size = strlen(data[i].value.strValue);
			return true;
			break;
		case TINT :
			snprintf(tmp,128,"%d",data[i].value.intValue);
			break;
		case TDOUBLE :
			snprintf(tmp,128,"%e",data[i].value.dblValue);
			break;
		case TBOOL :
			snprintf(tmp,128,"%s",
					data[i].value.boolValue?"true":"false");
			break;
	}
	*size = strlen(tmp);
	return true;
}

bool Config::write(const string & fname)  const
{
	FILE * fp = fopen(fname.c_str(),"w");
	if (fp == NULL) return false;
	print(fp,false,true);
	return true;
}
	
void Config::write(FILE * fp)  const
{
	print(fp,false,true);
}
	
static void escape_chars(string & s) 
{
	int i;
	char toreplace[6] = "\n\t\r\"$";
	char replacement[11] = "\\n\\t\\r\\\"\\$";
	string sought;
	for (i=0;i<5;i++) {
		sought.clear(); sought += toreplace[i];
		unsigned int pos = 0,where;
		while ((where = s.find(toreplace[i],pos)) != s.npos) {
			s.replace(where,1,replacement+2*i,2);
			pos = where + 2;
		}
	}
}

void Config::print(FILE * fp,bool pretty,bool hide_unaffected) const
{
	unsigned int i;
	Table::const_iterator itsection,ititems;
	for (itsection=cfg.begin();itsection!=cfg.end();itsection++) {
		const Table & T = sectiondata[itsection->second].section;
		if (hide_unaffected && (T.begin() != T.end()))
			fprintf(fp,"[ %s ]\n",itsection->first.c_str());
		for (ititems=T.begin(); ititems != T.end(); ititems++)
		{
			i = ititems->second;
			if (hide_unaffected && !data[i].affected) continue;
			if (pretty) {
				fprintf(fp,"%5u\t: %s ",data[i].id,data[i].name.c_str());
				switch (data[i].type)
				{
					case TINT : fprintf(fp,"(int) ");break;
					case TDOUBLE : fprintf(fp,"(double) ");break;
					case TSTRING : fprintf(fp,"(string) ");break;
					case TBOOL : fprintf(fp,"(bool) ");break;
					case TFREE : fprintf(fp,"(free) ");break;
					default : assert(false);
				}
				if (! data[i].affected)
					fprintf(fp,": unaffected\n");
				else 
				{
					switch (data[i].type)
					{
						case TINT : 
							fprintf(fp,"= %d\n",data[i].value.intValue);
							break;
						case TDOUBLE : 
							fprintf(fp,"= %e\n",data[i].value.dblValue);
							break;
						case TSTRING : 
							fprintf(fp,"= '%s'\n",data[i].value.strValue);
							break;
						case TBOOL : 
							fprintf(fp,"= %s\n",data[i].value.boolValue?"true":"false");
							break;
						default : 
							/* TFREE comes here, but should never exist
							 * affected */
							assert(false);
					}
				} 
			} else {
				string escaped;
				fprintf(fp,"%s ",data[i].name.c_str());
				switch (data[i].type)
				{
					case TINT : 
						fprintf(fp," = %d\n",data[i].value.intValue);
						break;
					case TDOUBLE : 
						fprintf(fp," = %e\n",data[i].value.dblValue);
						break;
					case TSTRING : 
						escaped.assign(data[i].value.strValue);
						escape_chars(escaped);
						fprintf(fp," = \"%s\"\n",escaped.c_str());
						break;
					case TBOOL : 
						fprintf(fp," = %s\n",data[i].value.boolValue?"true":"false");
						break;
					default : 
						assert(false);
				}
			}
		}
	}
}

bool Config::read(const string & fname)
{
	FILE * fp = fopen(fname.c_str(),"r");
	if (fp == NULL)
		return false;
	bool res = this->read(fp);
	fclose(fp);
	return res;
}


bool Config::read(FILE * fp)
{
	configin = fp;
	return ((configparse((void*)this) == 0) && 
			(ignoreInexistantID || !inexistantIdEncountered) &&
			(ignoreInvalidTypes || !invalidTypesEncountered)) ;
}
	
bool Config::exists(const string & name,const string & section_name)
{
	int i;
	if (!section_name.empty()) {
		pushSection();
		selectSection(section_name,false);
	}
	i = getIdx(name);
	if (!section_name.empty()) {
		popSection();
	}
	return (i >= 0);
}

int Config::getValue(const std::string & name, int defvalue) const
{
	int ret = defvalue;
	if (getInt(name,&ret)) {
		return ret;
	} else {
		return defvalue;
	}
}

bool Config::getValue(const std::string & name, bool defvalue) const
{
	bool ret = defvalue;
	if (getBool(name,&ret)) {
		return ret;
	} else {
		return defvalue;
	}
}

double Config::getValue(const std::string & name, double defvalue) const
{
	double ret = defvalue;
	if (getDouble(name,&ret)) {
		return ret;
	} else {
		return defvalue;
	}
}

std::string Config::getValue(const std::string & name, const std::string & defvalue) const
{
	std::string ret;
	if (getString(name,ret)) {
		return ret;
	} else {
		return defvalue;
	}
}

