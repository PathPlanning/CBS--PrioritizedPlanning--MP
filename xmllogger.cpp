#include "xmllogger.h"
#include <iostream>
#include <iomanip>

using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

bool XmlLogger::getLog(const char *FileName, const std::string *LogParams)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD) return true;

    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening input XML file" << std::endl;
        return false;
    }

    if (LogParams[CN_LP_PATH] == "" && LogParams[CN_LP_NAME] == "") {
        std::string str;
        str.append(FileName);
        size_t found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else if (LogParams[CN_LP_PATH] == "") {
        LogFileName.append(FileName);
        std::string::iterator it = LogFileName.end();
        while (*it != '\\')
            it--;
        ++it;
        LogFileName.erase(it, LogFileName.end());
        LogFileName.append(LogParams[CN_LP_NAME]);
    } else if (LogParams[CN_LP_NAME] == "") {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        std::string lfn;
        lfn.append(FileName);
        size_t found = lfn.find_last_of("\\");
        std::string str = lfn.substr(found);
        found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        LogFileName.append(LogParams[CN_LP_NAME]);
    }

    XMLElement *log, *root = doc.FirstChildElement(CNS_TAG_ROOT);

    if (!root) {
        std::cout << "No '" << CNS_TAG_ROOT << "' element found in XML file" << std::endl;
        std::cout << "Can not create log" << std::endl;
        return false;
    }

    /*tinyxml2::XMLDocument agentsDoc;
    if (agentsDoc.LoadFile(AgentsFileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening agents XML file" << std::endl;
        return false;
    }
    XMLElement *node, *agents, *agentsRoot = agentsDoc.FirstChildElement(CNS_TAG_ROOT);
    agents = doc.NewElement("agents");
    agents->SetAttribute("filename", AgentsFileName);
    for (node = agentsRoot->FirstChildElement(); node; node = node->NextSiblingElement()) {
        XMLElement *agent = doc.NewElement("agent");
        for (auto attribute = node->FirstAttribute(); attribute; attribute = attribute->Next()) {
            agent->SetAttribute(attribute->Name(), attribute->Value());
        }
        agents->InsertEndChild(agent);
    }
    root->InsertEndChild(agents);*/

    root->InsertEndChild(doc.NewElement(CNS_TAG_LOG));

    root = (root->LastChild())->ToElement();

    if (loglevel != CN_LP_LEVEL_NOPE_WORD) {
        log = doc.NewElement(CNS_TAG_MAPFN);
        log->LinkEndChild(doc.NewText(FileName));
        root->InsertEndChild(log);

        //root->InsertEndChild(doc.NewElement(CNS_TAG_SUM));

        //root->InsertEndChild(doc.NewElement(CNS_TAG_PATH));

        //root->InsertEndChild(doc.NewElement(CNS_TAG_PATHS));

        //root->InsertEndChild(doc.NewElement(CNS_TAG_LPLEVEL));

        //root->InsertEndChild(doc.NewElement(CNS_TAG_HPLEVEL));
    }

    if (loglevel == CN_LP_LEVEL_FULL_WORD || loglevel == CN_LP_LEVEL_MEDIUM_WORD)
        root->InsertEndChild(doc.NewElement(CNS_TAG_LOWLEVEL));

    return true;
}

void XmlLogger::saveLog()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;
    doc.SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogMap(const Map &map, const std::list<Node> &path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD)
        return;

    XMLElement *mapTag = doc.FirstChildElement(CNS_TAG_ROOT);
    mapTag = mapTag->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);

    int iterate = 0;
    bool inPath;
    std::string str;
    for (int i = 0; i < map.getMapHeight(); ++i) {
        XMLElement *element = doc.NewElement(CNS_TAG_ROW);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);

        for (int j = 0; j < map.getMapWidth(); ++j) {
            inPath = false;
            for(std::list<Node>::const_iterator it = path.begin(); it != path.end(); it++)
                if(it->i == i && it->j == j) {
                    inPath = true;
                    break;
                }
            if (!inPath)
                str += std::to_string(map.getValue(i,j));
            else
                str += CNS_OTHER_PATHSELECTION;
            str += CNS_OTHER_MATRIXSEPARATOR;
        }

        element->InsertEndChild(doc.NewText(str.c_str()));
        mapTag->InsertEndChild(element);
        str.clear();
        iterate++;
    }
}

void XmlLogger::writeToLogAgentsPaths(const AgentSet& agentSet,
                                      const std::vector<std::vector<Node>>& agentsPaths,
                                      const std::string &agentsFile, double time,
                                      double makespan, double flowtime,
                                      int HLExpansions, int HLNodes,
                                      double LLExpansions, double LLNodes, int scale,
                                      bool breakPrimitives, int step, const Primitives& mp) {
    XMLElement *log = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);

    XMLElement *taskFileElement = doc.NewElement(CNS_TAG_TASKFN);
    taskFileElement->SetText(agentsFile.c_str());
    log->InsertEndChild(taskFileElement);

    XMLElement *summaryElement = doc.NewElement(CNS_TAG_SUMMARY);
    summaryElement->SetAttribute(CNS_TAG_ATTR_COUNT, int(agentsPaths.size()));
    summaryElement->SetAttribute(CNS_TAG_ATTR_TIME, time);
    summaryElement->SetAttribute(CNS_TAG_ATTR_MAKESPAN, makespan);
    summaryElement->SetAttribute(CNS_TAG_ATTR_FLOWTIME, flowtime);
    summaryElement->SetAttribute(CNS_TAG_ATTR_HLE, HLExpansions);
    summaryElement->SetAttribute(CNS_TAG_ATTR_HLN, HLNodes);
    summaryElement->SetAttribute(CNS_TAG_ATTR_LLE, LLExpansions);
    summaryElement->SetAttribute(CNS_TAG_ATTR_LLN, LLNodes);
    log->InsertEndChild(summaryElement);

    for (int i = 0; i < agentsPaths.size(); ++i) {
        int id = 1;
        Agent agent = agentSet.getAgent(i);
        XMLElement *agentElement = doc.NewElement(CNS_TAG_AGENT);
        agentElement->SetAttribute(CNS_TAG_ATTR_ID, i);
        agentElement->SetAttribute(CNS_TAG_ATTR_STARTX, agent.getStart_j() / scale);
        agentElement->SetAttribute(CNS_TAG_ATTR_STARTY, agent.getStart_i() / scale);
        agentElement->SetAttribute(CNS_TAG_ATTR_GOALX, agent.getGoal_j() / scale);
        agentElement->SetAttribute(CNS_TAG_ATTR_GOALY, agent.getGoal_i() / scale);

        XMLElement *pathElement = doc.NewElement(CNS_TAG_PATH);
        pathElement->SetAttribute(CNS_TAG_ATTR_PATH_FOUND, "true");

        for (int j = 1; j < agentsPaths[i].size(); ++j) {
            std::vector<std::tuple<double, double, double, double>> positions;
            auto pr = mp.getPrimitive(agentsPaths[i][j].primitiveId);
            if (breakPrimitives && agentsPaths[i][j - 1] != agentsPaths[i][j]) {
                pr.getPositions(positions, agentsPaths[i][j - 1],
                        agentsPaths[i][j - 1].g, agentsPaths[i][j].g, step, 100);
            } else {
                positions = {std::make_tuple(agentsPaths[i][j - 1].i, agentsPaths[i][j - 1].j, 0, Primitive::idToAngle(agentsPaths[i][j - 1].angleId)),
                             std::make_tuple(agentsPaths[i][j].i, agentsPaths[i][j].j,
                             double(agentsPaths[i][j].g - agentsPaths[i][j - 1].g) / 1000,
                             Primitive::idToAngle(agentsPaths[i][j].angleId))};
            }

            for (int k = 0; k < positions.size() - 1; ++k) {
                XMLElement *sectionElement = doc.NewElement(CNS_TAG_SECTION);
                Point curPoint = Point(std::get<0>(positions[k]), std::get<1>(positions[k]));
                Point nextPoint = Point(std::get<0>(positions[k + 1]), std::get<1>(positions[k + 1]));
                sectionElement->SetAttribute(CNS_TAG_ATTR_ID, id++);
                sectionElement->SetAttribute(CNS_TAG_ATTR_STARTX, curPoint.j / scale);
                sectionElement->SetAttribute(CNS_TAG_ATTR_STARTY, curPoint.i / scale);
                sectionElement->SetAttribute(CNS_TAG_ATTR_GOALX, nextPoint.j / scale);
                sectionElement->SetAttribute(CNS_TAG_ATTR_GOALY, nextPoint.i / scale);
                sectionElement->SetAttribute(CNS_TAG_ATTR_STARTH, std::get<3>(positions[k]) * 180 / CN_PI);
                sectionElement->SetAttribute(CNS_TAG_ATTR_GOALH, std::get<3>(positions[k + 1]) * 180 / CN_PI);
                sectionElement->SetAttribute(CNS_TAG_ATTR_DUR, std::get<2>(positions[k + 1]) - std::get<2>(positions[k]));
                pathElement->InsertEndChild(sectionElement);
            }
        }
        agentElement->InsertEndChild(pathElement);
        log->InsertEndChild(agentElement);
    }
}

void XmlLogger::writeToLogAggregatedResults(std::map<int, int>& successCount,
                                            TestingResults &res,
                                            const std::string& agentsFile) {
    XMLElement *log = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);
    XMLElement *results = doc.NewElement(CNS_TAG_RESULTS);
    if (!agentsFile.empty()) {
        results->SetAttribute(CNS_TAG_AGENTS_FILE, agentsFile.c_str());
    }
    std::vector<std::string> keys = res.getKeys();
    for (auto pair : successCount) {
        XMLElement *result = doc.NewElement(CNS_TAG_RESULT);
        result->SetAttribute(CNS_TAG_ATTR_COUNT, pair.first);
        result->SetAttribute(CNS_TAG_ATTR_SC, pair.second);
        for (auto key : keys) {
            result->SetAttribute(key.c_str(), res.data[key][pair.first]);
        }
        results->InsertEndChild(result);
    }
    log->InsertEndChild(results);
}

/*void XmlLogger::writeToLogOpenClose(const typename &open, const typename &close)
{
    //need to implement
    if (loglevel != CN_LP_LEVEL_FULL_WORD  && !(loglevel == CN_LP_LEVEL_MEDIUM_WORD && last))
        return;


}*/

void XmlLogger::writeToLogPath(const std::list<Node> &path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || path.empty())
        return;
    int iterate = 0;
    XMLElement *lplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lplevel = lplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LPLEVEL);

    for (std::list<Node>::const_iterator it = path.begin(); it != path.end(); it++) {
        XMLElement *element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, it->j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
        lplevel->InsertEndChild(element);
        iterate++;
    }
}

void XmlLogger::writeToLogHPpath(const std::list<Node> &hppath)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || hppath.empty())
        return;
    int partnumber = 0;
    XMLElement *hplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    hplevel = hplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_HPLEVEL);
    std::list<Node>::const_iterator iter = hppath.begin();
    std::list<Node>::const_iterator it = hppath.begin();

    while (iter != --hppath.end()) {
        XMLElement *part = doc.NewElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_ATTR_STX, it->j);
        part->SetAttribute(CNS_TAG_ATTR_STY, it->i);
        ++iter;
        part->SetAttribute(CNS_TAG_ATTR_FINX, iter->j);
        part->SetAttribute(CNS_TAG_ATTR_FINY, iter->i);
        part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
        hplevel->LinkEndChild(part);
        ++it;
        ++partnumber;
    }
}

void XmlLogger::writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement *summary = doc.FirstChildElement(CNS_TAG_ROOT);
    summary = summary->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_SUM);
    XMLElement *element = summary->ToElement();
    element->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
    element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH, length);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH_SCALED, length*cellSize);
    element->SetAttribute(CNS_TAG_ATTR_TIME, std::to_string(time).c_str());
}

void XmlLogger::writeToLogNotFound()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement *node = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);
    node->InsertEndChild(doc.NewText("Path NOT found!"));
}

//void XmlLogger::writeToLogPushAndRotateResult(std::vector<AgentPosition> result)
//{
//    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
//        return;

//    XMLElement *node = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);
//    node->InsertEndChild(doc.NewText("Path NOT found!"));
//}
