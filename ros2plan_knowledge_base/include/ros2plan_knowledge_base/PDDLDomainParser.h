#ifndef KCL_domainparser
#define KCL_domainparser

#include <vector>
#include <iostream>
#include <fstream>

#include "VALfiles/ptree.h"
#include "FlexLexer.h"

extern int yyparse();
extern int yydebug;

/**
 * Domain storage and parsing. Uses VAL parser and storage.
 */
namespace KCL_ros2plan {

	/*--------*/
	/* parser */
	/*--------*/

	class PDDLDomainParser
	{
	private:

	public:

		/* VAL pointers */
		VAL1_2::analysis * val_analysis;
		VAL1_2::domain* domain;

		/* domain information */
		bool domain_parsed;
		std::string domain_name;

		/* domain parsing */
		VAL1_2::domain* parseDomain(const std::string domainPath);
	};
}
#endif
