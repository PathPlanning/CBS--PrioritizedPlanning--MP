#ifndef CONFIG_H
#define	CONFIG_H
#include <string>
#include "tinyxml2.h"

class Config
{
    public:
        Config();
        Config(const Config& orig);
        ~Config();
        bool getConfig(const char *FileName);

        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;
        int             searchType;
        int             lowLevel;
        int             minAgents = 1;
        int             maxAgents = -1;
        int             maxTime = 1000;
        std::string     agentsFile;
        int             tasksCount = 1;
        int             neighDegree = 2;
        bool            withCAT = false;
        bool            withPerfectHeuristic = false;
        int             ppOrder = 0;
        bool            parallelizePaths1 = false;
        bool            parallelizePaths2 = false;
        bool            singleExecution = false;
        bool            withCardinalConflicts = false;
        bool            withBypassing = false;
        bool            withMatchingHeuristic = false;
        bool            storeConflicts = false;
        bool            withDisjointSplitting;
        bool            withFocalSearch = false;
        bool            genSuboptFromOpt = false;
        bool            saveAggregatedResults = true;
        double          focalW = 1.0;

    private:
        bool getValueFromText(tinyxml2::XMLElement *elem, const char *name, const char *typeName, void *field);
        bool getValueFromAttribute(tinyxml2::XMLElement *elem, const char *elemName,
                                          const char *attrName, const char *typeName, void *field);
        bool getText(tinyxml2::XMLElement *elem, const char *name, std::string &field);
        tinyxml2::XMLElement* getChild(tinyxml2::XMLElement *elem, const char *name, bool printError = true);
};

#endif

