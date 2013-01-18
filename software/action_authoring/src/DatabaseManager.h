#pragma once
#include "Constraint.h"
#include "Affordance.h"
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <libxml/parser.h>
#include <libxml/tree.h>

namespace action_authoring {
	class DatabaseManager {
		protected:
			//member variables
			const char* m_filename;
			int m_guidCounter;
			std::map<Affordance*, int> m_affordanceToGUID;
			std::map<Constraint*, int> m_constraintToGUID;
			std::vector<Affordance*> m_affordanceList;
			std::vector<Constraint*> m_constraintList;

			//writing to file helper functions
			std::string intToString(int i);
			void addAffordanceToNode(Affordance* affordance, xmlNodePtr node);
			void addConstraintToNode(Constraint* constrain, xmlNodePtr node);
			void postOrderAddConstraintToQueue(Constraint* constraint, std::queue<Constraint*>* q, std::set<Constraint*>* done);
			int getGUID(Constraint* constraint);
			int getGUID(Affordance* affordance);

			//reading from file helper functions
			void parseTreeHelper(xmlDocPtr doc, xmlNode* xmlnode, std::map<int,  Affordance*>* affordances, std::map<int, Constraint*>* constraints);
			void parseTree(xmlDocPtr doc, xmlNode* root);

		public:
			//constructor
			DatabaseManager(const char* filename);

			//mutators
			void setFilename(const char* filename);

			//Writing data to a file
			void store(std::vector<Affordance*> affordanceList, std::vector<Constraint*> constrainList);

			//Reading data from a file
			void parseFile();
			std::vector<Affordance*> getAffordances();
			std::vector<Constraint*> getConstraints();
	};
}