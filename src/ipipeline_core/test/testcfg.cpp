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

#include "ipipeline_core/Config.h"



int main(int argc,char * argv[])
{

	Config cfgreader;
	if (argc<2)
		return 1;
		
	FILE * fp = fopen(argv[1],"r");
	if (fp == NULL)
		return 2;
	
	bool r = cfgreader.read(fp);
	fflush(stderr);
	fflush(stdout);

	fclose(fp);
	if (!r) {
		printf("Warning : error encountered while reading %s\n",argv[1]);
		printf("-------------------\n");
	}
	

	cfgreader.print(stdout,true,true);
	std::string dest(argv[1]);
	cfgreader.write(dest + ".bak");
	return 0;
}




