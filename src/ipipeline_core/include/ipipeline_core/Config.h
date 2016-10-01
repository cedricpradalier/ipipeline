/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#ifndef CONFIG_H
#define CONFIG_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>
#include <list>
#include <string>


/** \enum ConfigType
 * Type de donnee possibles des parametres de configuration
 * */
typedef enum {TFREE,TINT,TDOUBLE,TSTRING,TBOOL} ConfigType;

/** Classe Config
 *
 *
 * */
class Config
{
	private :
		struct ConfigItem
		{
			bool affected;
			unsigned int id;
			std::string name;
			ConfigType type;
			union {
				int intValue;
				double dblValue;
				char * strValue;
				bool boolValue;
			} value;

			ConfigItem() {}
			ConfigItem(const ConfigItem &c) {
				name = c.name;
				affected = c.affected;
				id = c.id;
				type = c.type;
				value = c.value;
				if (affected && (type == TSTRING)) {
					value.strValue = strdup(c.value.strValue);
				}
			}
			~ConfigItem() {
				if (affected && (type == TSTRING)) {
					free(value.strValue);
				}
			}
		};
		std::vector<ConfigItem> data;
		
		typedef std::map<std::string,unsigned int,std::less<std::string> > Table;

		struct SectionItem 
		{
			std::string name;
			unsigned int id;
			Table section;
		};
		std::vector<SectionItem> sectiondata;
		std::list<unsigned int> pushed_section;

		Table cfg;
		unsigned int currentTable;

		int getIdx(const std::string& name,bool create);
		int getIdx(const std::string& name) const;

		bool affectInt(unsigned int i,int a);
		bool affectDouble(unsigned int i,double a);
		bool affectString(unsigned int i,const std::string & a);
		bool affectBool(unsigned int i,bool b);
		bool affectEnv(const std::string& name,const std::string& id_src,bool create=true);

		bool ignoreInexistantID;
		bool ignoreInvalidTypes;

	public :

		/** Constructeur */
		Config();
		/** Destructeur*/
		~Config();

		const Config & operator=(const Config & config) {
			data = config.data;
			sectiondata = config.sectiondata;
			cfg = config.cfg;
			pushed_section.clear();
			return *this;
		}

		

		/** 
		*  Definit un nouvel item : name, de type t
		*/
		unsigned int addDefinition(const std::string & name,ConfigType t,
				const std::string & class_name = std::string());

		/**
		 * Doit on considerer que la lecture a echoue 
		 * quand le fichier contient des IDs inconnus ?
		 * */
		void failWithInexistantID(bool v) {ignoreInexistantID=!v;}
		
		/**
		 * Doit on considerer que la lecture a echoue 
		 * quand le fichier contient des types invalides ?
		 * */
		void failWithInvalidTypes(bool v) {ignoreInvalidTypes=!v;}

		/** 
		*  Lecture d'une config dans le fichier fname
		*/
		bool read(const std::string & fname);

		/** 
		*  Lecture d'une config dans fp. 
		*  Precondition : fp a ete ouvert en lecture
		*/
		bool read(FILE * fp);

		/** 
		*  Ecriture d'une config dans fp. 
		*  Precondition : fp a ete ouvert en ecriture
		*/
		bool write(const std::string & fname) const;
		void write(FILE * fp) const;

		/** 
		*  Affiche une config dans fp
		*  Precondition : fp a ete ouvert en ecriture
		*/
		void print(FILE * fp = stdout,bool pretty = true, bool hide_unaffected = false) const;

		
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', le resultat est stockee
		*  dans *value et la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool getInt(const std::string & name,int * value) const;
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', le resultat est stockee
		*  dans *value et la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool getUInt(const std::string & name,unsigned int * value) const;
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', le resultat est stockee
		*  dans *value et la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool getDouble(const std::string & name,double * value) const;
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', le resultat est stockee
		*  dans *value et la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool getBool(const std::string & name,bool * value) const;
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', le resultat est stockee
		*  dans *value et la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*  ATTENTION : *value est alloue dans cette fonction par 
		*  strdup(). N'oubliez pas de le liberer avec free().
		*/
		bool getStringDup(const std::string & name,char ** value) const;
		
		/** 
		*  Demande la longueur(strlen) de la chaine associee a 
		*  name. Si name est un item valide et affecte', 
		*  le resultat est stockee dans *size.
		*  La fonction retourne false dans le cas contraire
		*/
		bool getStringLength(const std::string & name,unsigned int * siz) const;
		
		/** 
		*  Demande la valeur associee a name. Si name est
		*  un item valide et affecte', et si la longueur de 
		*  la chaine est inferieure (strlen) a maxsize,
		*  le resultat est stockee dans value et la fonction 
		*  retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool getString(const std::string & name,char * value,unsigned int maxsize) const;
		bool getString(const std::string & name,std::string & output) const;

		/** Polymorph function to get the value of a config element **/
		int getValue(const std::string & name, int defvalue) const;
		bool getValue(const std::string & name, bool defvalue) const;
		double getValue(const std::string & name, double defvalue) const;
		std::string getValue(const std::string & name, const std::string & defvalue) const;

		/** 
		*  Affecte la valeur associee a name a partir de la valeur associee a
		*  id_src. La fonction retourne true si
		*  <ul> 
		*  <li> name est un item valide </li>
		*  <li> id_src est soit un item valide, soit une variable
		*  d'environnement valide.</li>
		*  <li> la valeur de id_src est connue. </li>
		*  <li> la valeur associee a id_src correspond au type de name.</li>
		*  </ul>
		*  La fonction retourne false dans le cas contraire
		*/
		bool affectID(const std::string & name,const std::string & id_section,
				const std::string & id_name,bool create = true);
		/** 
		*  Affecte la valeur associee a name. Si name est
		*  un item valide,  la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool affectInt(const std::string & name,int a,bool create = true);
		/** 
		*  Affecte la valeur associee a name. Si name est
		*  un item valide,  la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool affectDouble(const std::string & name,double a,bool create = true);
		/** 
		*  Affecte la valeur associee a name. Si name est
		*  un item valide,  la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool affectString(const std::string & name,const char * a,bool create = true);
		bool affectString(const std::string & name,const std::string & a,bool create = true);
		/** 
		*  Affecte la valeur associee a name. Si name est
		*  un item valide,  la fonction retourne true. 
		*  La fonction retourne false dans le cas contraire
		*/
		bool affectBool(const std::string & name,bool b,bool create = true);

		/** 
		 * Teste si name est un nom definit dans cette config
		 */
		bool exists(const std::string & name,const std::string & section_name) ;

		void pushSection();
		bool popSection();

		/**
		 * Selectionne le contexte section_name
		 * */
		bool selectSection(const std::string & section_name,bool create = false);

		const std::string& currentSection() const {return sectiondata[currentTable].name;}

		void clear() {
			data.clear();
			sectiondata.clear();
			cfg.clear();
			pushed_section.clear();
		}
};


#endif // CONFIG_H
