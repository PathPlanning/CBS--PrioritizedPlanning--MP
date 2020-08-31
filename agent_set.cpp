#include "agent_set.h"

void AgentSet::clear() {
    occupiedNodes.clear();
    agents.clear();
}

void AgentSet::addAgent(int start_i, int start_j, int goal_i, int goal_j) {
    occupiedNodes[std::make_pair(start_i, start_j)] = agents.size();
    agents.push_back(Agent(start_i, start_j, goal_i, goal_j, agents.size()));
}

int AgentSet::getAgentCount() const {
    return agents.size();
}

Agent AgentSet::getAgent(int i) const {
    return agents[i];
}

bool AgentSet::isOccupied(int i, int j) const {
    return occupiedNodes.find(std::make_pair(i, j)) != occupiedNodes.end();
}

int AgentSet::getAgentId(int i, int j) const {
    return occupiedNodes.at(std::make_pair(i, j));
}

int AgentSet::rescale(int x, int scale) {
    return x * scale + scale / 2;
}

bool AgentSet::readAgents(const char *FileName, int scale)
{
    tinyxml2::XMLElement *root = 0, *node;
    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (node = root->FirstChildElement(); node; node = node->NextSiblingElement()) {
        int id, start_i, start_j, goal_i, goal_j;
        node->QueryIntAttribute("id", &id);
        node->QueryIntAttribute("start_i", &start_i);
        node->QueryIntAttribute("start_j", &start_j);
        node->QueryIntAttribute("goal_i", &goal_i);
        node->QueryIntAttribute("goal_j", &goal_j);
        addAgent(rescale(start_i, scale), rescale(start_j, scale),
                 rescale(goal_i, scale), rescale(goal_j, scale));
    }

    return true;
}
