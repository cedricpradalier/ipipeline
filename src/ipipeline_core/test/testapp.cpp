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


	cfgreader.addDefinition("string1",TSTRING,"strings");
	cfgreader.addDefinition("string2",TSTRING,"strings");
	cfgreader.addDefinition("string3",TSTRING,"strings");
	cfgreader.addDefinition("string4",TSTRING,"strings");


	cfgreader.addDefinition("int1",TINT,"ints");
	cfgreader.addDefinition("int2",TINT,"ints");
	cfgreader.addDefinition("int3",TINT,"ints");
	cfgreader.addDefinition("int4",TINT,"ints");

	cfgreader.addDefinition("bool1",TBOOL,"bools");
	cfgreader.addDefinition("bool2",TBOOL,"bools");
	cfgreader.addDefinition("bool3",TBOOL,"bools");
	cfgreader.addDefinition("bool4",TBOOL,"bools");
	cfgreader.addDefinition("bool5",TBOOL,"bools");
	cfgreader.addDefinition("bool6",TBOOL,"bools");
	cfgreader.addDefinition("bool7",TBOOL,"bools");
	cfgreader.addDefinition("bool8",TBOOL,"bools");
	cfgreader.addDefinition("bool9",TBOOL,"bools");

	cfgreader.addDefinition("double1",TDOUBLE,"doubles");
	cfgreader.addDefinition("double2",TDOUBLE,"doubles");
	cfgreader.addDefinition("double3",TDOUBLE,"doubles");
	cfgreader.addDefinition("double4",TDOUBLE,"doubles");
	cfgreader.addDefinition("double5",TDOUBLE,"doubles");
	cfgreader.addDefinition("double6",TDOUBLE,"doubles");

	if (argc<2)
		return 1;
		
	FILE * fp = fopen(argv[1],"r");
	if (fp == NULL)
		return 2;
	
	bool r = cfgreader.read(fp);
	fflush(stderr);
	fflush(stdout);

	fclose(fp);
	if (!r)
		printf("Warning : error encountered while reading %s\n",argv[1]);
	

	printf("-------------------\n");

	
	cfgreader.print(stdout,true,false);
	cfgreader.write(stdout);
	char tmp[1024];
	sprintf(tmp,"%s.bak",argv[1]);
	fp = fopen(tmp,"w");
	cfgreader.write(fp);
	fclose(fp);
	return 0;
}




