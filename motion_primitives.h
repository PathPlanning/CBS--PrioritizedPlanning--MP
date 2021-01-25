#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include "tinyxml2.h"
#include "map.h"
#include "math.h"

class Position
{
    public:
    int i;
    int j;
    int angle_id;
    int speed;
};

class Cell
{
    public:
    Cell(int i_, int j_):i(i_), j(j_){}
    int i;
    int j;
    std::pair<int, int> interval;
    bool operator <(const Cell& other) const
    {
        if(i == other.i)
            return j < other.j;
        return i < other.i;
    }
    bool operator ==(const Cell& other) const
    {
        if(i == other.i && j == other.j)
            return true;
        else
            return false;
    }
};

class Point {
    public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;//LEFT;
        if (sa < 0.0)
            return 2;//RIGHT;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;//BEHIND;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;//BEYOND;
        if (pO == p2)
            return 5;//ORIGIN;
        if (p1 == p2)
            return 6;//DESTINATION;
        return 7;//BETWEEN;
    }
    double minDist(Point C, Point D)
    {
        int classA = this->classify(C, D);
        if(classA == 3)
            return sqrt(pow(this->i - C.i, 2) + pow(this->j - C.j, 2));
        else if(classA == 4)
            return sqrt(pow(this->i - D.i, 2) + pow(this->j - D.j, 2));
        else
            return fabs((C.i - D.i)*this->j + (D.j - C.j)*this->i + (C.j*D.i - D.j*C.i))/sqrt(pow(C.i - D.i, 2) + pow(C.j - D.j, 2));
    }
};


struct IntervalBoundary {
    int time, id;
    bool end;

    IntervalBoundary(int Time = 0, int Id = 0, bool End = false) : time(Time), id(Id), end(End) {}

    bool operator<(const IntervalBoundary &other) const {
        return std::make_tuple(time, end, id) < std::make_tuple(other.time, other.end, other.id);
    }
};


class Primitive
{
public:
    int id;
    int type;
    double begin;
    double duration;
    int intDuration;
    Position source;
    Position target;
    double agentSize;
    std::vector<double> i_coefficients;
    std::vector<double> j_coefficients;
    std::vector<Cell> cells;
    bool operator==(const Primitive& other) { return this->id == other.id; }
    void setSize(double size) { agentSize = size; }
    std::vector<Cell> getCells() const { return cells; }
    double size() {return agentSize;}
    void setSource(int i, int j)
    {
        source.i = i;
        source.j = j;
        target.i = i + target.i;
        target.j = j + target.j;
        for(auto &c:cells)
        {
            c.i = i + c.i;
            c.j = j + c.j;
        }
    }

    void setStartTime(int startTime)
    {
        for(auto &c:cells)
        {
            c.interval.first += startTime;
            c.interval.second += startTime;
        }
    }

    double clip(double begin, double end, double val) {
        return std::max(begin, std::min(val, end));
    }

    double getEndpoint(Cell cell, double begin, double end, double resolution, double size, bool start)
    {
        if(resolution < CN_RESOLUTION)
        {
            if(start && begin + resolution*10 > duration - CN_EPSILON)
                return -1;
            else
                return start?begin:begin+resolution;
        }
        double cur_t(begin);
        while(true)
        {
            Point pos = this->getPos(cur_t);
            double closest_i = clip(double(cell.i) - size, double(cell.i) + size, pos.i);
            double closest_j = clip(double(cell.j) - size, double(cell.j) + size, pos.j);
            double dist = pow(pos.i - closest_i, 2) + pow(pos.j - closest_j, 2);
            if((start && dist < pow(agentSize, 2)) || (!start && dist > pow(agentSize, 2)))
            {
                if(cur_t - resolution < 0)
                    return 0;
                return getEndpoint(cell, cur_t - resolution, std::min(cur_t, duration), resolution/10, size, start);
            }
            cur_t += resolution;
            if(cur_t > end - CN_EPSILON)
            {
                if(resolution > CN_RESOLUTION)
                    return getEndpoint(cell, cur_t - resolution, std::min(cur_t, duration), resolution/10, size, start);
                else
                    return start?-1:end;
            }
        }
    }
    std::pair<int, int> getInterval(int i, int j)
    {
        auto it = std::find(cells.begin(), cells.end(), Cell(i,j));
        if(it == cells.end())
            return {-1,-1};
        else if(it->interval.first < 0)
            return {-1,-1};
        return {begin + it->interval.first, begin + it->interval.second};
    }

    double getAngle(double t, bool orth = true)
    {
        if(type < 0)
            return -1;
        double num = i_coefficients[1]+2*i_coefficients[2]*t + 3*i_coefficients[3]*t*t;
        double denum = j_coefficients[1]+2*j_coefficients[2]*t+3*j_coefficients[3]*t*t;
        if (!orth) {
            std::swap(num, denum);
        }
        return atan2(num, denum);
    }
    Point getPos(double t)
    {
        if(type < 0)
            return Point(0,0);
        Point p;
        p.j = j_coefficients[0] + j_coefficients[1]*t + j_coefficients[2]*t*t + j_coefficients[3]*t*t*t;
        p.i = i_coefficients[0] + i_coefficients[1]*t + i_coefficients[2]*t*t + i_coefficients[3]*t*t*t;
        return p;
    }
    int doubleToInt(double val, int time_resolution) {
        return int(val * time_resolution);
    }

    void countIntervals(double size, int time_resolution)
    {
        double r = size + agentSize;

        std::vector<Cell> newCells;
        for(auto &c:cells)
        {
            double start = getEndpoint(c, 0, duration, 0.01, size, true);
            c.interval.first = doubleToInt(start, time_resolution);
            if(c.interval.first < 0)
                continue;
            c.interval.second = doubleToInt(getEndpoint(c, start + CN_RESOLUTION*2,
                                                        duration, 0.01, size, false), time_resolution);
            if(c.interval.first > c.interval.second)
                continue;

            newCells.push_back(c);
        }
        cells = newCells;
    }
    void countCells()
    {
        double t(0);
        bool stop(false);
        while(t < duration + CN_EPSILON)
        {
            double angle = getAngle(t);
            double gap_i = cos(angle)*0.5;
            double gap_j = sin(angle)*0.5;
            auto p = getPos(t);
            int c_i(p.i+gap_i+0.5-1e-3);
            if(p.i+gap_i < 0)
                c_i = p.i+gap_i-0.5+1e-3;
            int c_j(p.j-gap_j+0.5-1e-3);
            if(p.j-gap_j < 0)
                c_j = p.j-gap_j-0.5+1e-3;
            Cell c(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            c_i = p.i-gap_i+0.5-1e-3;
            if(p.i-gap_i < 0)
                c_i = p.i-gap_i-0.5+1e-3;
            c_j = p.j+gap_j+0.5-1e-3;
            if(p.j+gap_j < 0)
                c_j = p.j+gap_j-0.5+1e-3;
            c = Cell(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            c_i = p.i+0.5-1e-3;
            if(p.i < 0)
                c_i = p.i-0.5+1e-3;
            c_j = p.j+0.5-1e-3;
            if(p.j < 0)
                c_j = p.j-0.5+1e-3;
            c = Cell(c_i, c_j);
            if(std::find(cells.begin(), cells.end(), c) == cells.end())
                cells.push_back(c);

            if(t + 0.1 >= duration)
            {
                if(stop)
                    break;
                t = duration;
                stop = true;
            }
            else
                t += 0.1;
        }
    }

   static double idToAngle(int id) {
        double res = CN_PI / 2 - CN_PI / 4 * id;
        if (res < -CN_PI) {
            res += 2 * CN_PI;
        }
        return res;
    }

    void getPositions(std::vector<std::tuple<double, double, double, double>>& positions,
                      const Node& start, int startTime, int endTime, int step, int resolution) {
        int waitTime = endTime - startTime - intDuration;
        for (int time = 0; time < waitTime; time += step) {
            positions.push_back(std::make_tuple(start.i, start.j, double(time) / resolution,
                                                idToAngle(source.angle_id)));
        }
        for (int time = waitTime; time + startTime < endTime; time += step) {
            auto p = getPos(double(time - waitTime) / resolution);
            double angle;
            if (time == 0) {
                angle = idToAngle(source.angle_id);
            } else {
                angle = getAngle(double(time - waitTime) / resolution, false);
            }
            positions.push_back(std::make_tuple(p.i + start.i, p.j + start.j, double(time) / resolution, angle));
        }
        positions.push_back(std::make_tuple(target.i + start.i, target.j + start.j,
                                            double(endTime - startTime) / resolution,
                                            idToAngle(target.angle_id)));
    }
};

class Primitives
{
    public:
    std::vector<std::vector<Primitive>> type0;
    std::vector<std::vector<Primitive>> type1;
    std::vector<std::vector<Primitive>> type2;
    int time_resolution;
    bool loadPrimitives(const char* FileName, int Time_resolution, int scale, double agentSize)
    {
        time_resolution = Time_resolution;
        std::string value;
        std::stringstream stream;

        tinyxml2::XMLDocument doc;
        if(doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
        {
            std::cout << "Error openning input XML file."<<std::endl;
            return false;
        }

        tinyxml2::XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
        if (!root)
        {
            std::cout << "No 'root' element found in XML file."<<std::endl;
            return false;
        }
        int id=0;
        for(tinyxml2::XMLElement  *elem = root->FirstChildElement();elem;elem = elem->NextSiblingElement("trajectory"))
        {
            for(tinyxml2::XMLElement  *coef = elem->FirstChildElement();coef; coef = coef->NextSiblingElement("coeff"))
            {
                Primitive prim;
                prim.id = coef->IntAttribute("id");
                id++;
                prim.type = coef->IntAttribute("v0");

                prim.duration = coef->DoubleAttribute("Tf");
                prim.intDuration = prim.doubleToInt(prim.duration, time_resolution);
                prim.agentSize = agentSize;

                prim.source.i = 0;
                prim.source.j = 0;
                prim.source.angle_id = coef->IntAttribute("phi0")/45;
                prim.source.speed = coef->IntAttribute("v0");

                prim.target.i = coef->IntAttribute("yf");
                prim.target.j = coef->IntAttribute("xf");
                prim.target.angle_id = coef->IntAttribute("phif")/45;
                prim.target.speed = coef->IntAttribute("vf");

                prim.i_coefficients.push_back(coef->DoubleAttribute("b1"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b2"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b3"));
                prim.i_coefficients.push_back(coef->DoubleAttribute("b4"));

                prim.j_coefficients.push_back(coef->DoubleAttribute("a1"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a2"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a3"));
                prim.j_coefficients.push_back(coef->DoubleAttribute("a4"));

                prim.countCells();
                prim.countIntervals(0.5, time_resolution);
                if(prim.type == 0)
                {
                    if(type0.empty() || type0.back().at(0).source.angle_id != prim.source.angle_id)
                        type0.push_back({prim});
                    else
                        type0.back().push_back(prim);
                }
                else
                {
                    if(type1.empty() || type1.back().at(0).source.angle_id != prim.source.angle_id)
                        type1.push_back({prim});
                    else
                        type1.back().push_back(prim);
                }
            }
            type2.push_back({});
            for(tinyxml2::XMLElement  *turning = elem->FirstChildElement("time_finish"); turning; turning = turning->NextSiblingElement("time_finish"))
            {
                Primitive prim;
                prim.type = -1;
                prim.id = turning->IntAttribute("id");
                prim.source.i = prim.source.j = prim.target.i = prim.target.j = prim.source.speed = prim.target.speed = 0;
                prim.source.angle_id = turning->IntAttribute("phi0")/45;
                prim.target.angle_id = turning->IntAttribute("phif")/45;
                prim.duration = turning->DoubleAttribute("Tf");
                prim.intDuration = prim.doubleToInt(prim.duration, time_resolution);
                prim.agentSize = agentSize;
                prim.cells = {Cell(0,0)};
                prim.cells[0].interval = {0, prim.intDuration};
                type2.back().push_back(prim);
            }
        }
    }

    int getId() {
        int res = 0;
        for (auto type : {type0, type1, type2}) {
            for (auto pr : type) {
                res += pr.size();
            }
        }
        return res;
    }

    void addWaitPrimitive()
    {
        Primitive pr;
        pr.intDuration = 1;
        pr.source.speed = 0;
        pr.target.speed = 0;
        pr.target.i = 0;
        pr.target.j = 0;
        pr.id = getId();
        pr.cells.push_back(Cell(0, 0));
        pr.cells[0].interval = std::make_pair(0, 1);
        if (type2.empty()) {
            type2.push_back({});
        }
        for (int k = 0; k < type2.size(); ++k) {
            pr.source.angle_id = k;
            pr.target.angle_id = k;
            type2[k].push_back(pr);
        }
    }

    Primitive getPrimitive(int id) const
    {
        for(auto p:type0)
            for(auto t:p)
                if(t.id == id)
                    return t;
        for(auto p:type1)
            for(auto t:p)
                if(t.id == id)
                    return t;
        for(auto p:type2)
            for(auto t:p)
                if(t.id == id)
                    return t;
    }
    void getPrimitives(std::vector<Primitive> &prims, int i, int j, int angle_id, int speed, const Map& map)
    {
        int size = (speed == 0) ? type0[angle_id].size() : type1[angle_id].size();
        for(int k = 0; k < size; k++) {
            Primitive prim = (speed == 0) ? type0[angle_id][k] : type1[angle_id][k];
            if (map.CellOnGrid(i + prim.target.i, j + prim.target.j)) {
                bool good = true;
                for(auto c : prim.getCells()) {
                    if(c.interval.first > -CN_EPSILON) {
                        if (map.CellIsObstacle(i+c.i, j+c.j)) {
                            good = false;
                            break;
                        }
                    }
                }
                if (good) {
                    prims.push_back(prim);
                }
            }
        }
    }

    bool withTurns() {
        return type0.size() > 1;
    }

    int orientationCount() const {
        return type0.size();
    }

    void getTurns(std::vector<Primitive> &prims, int angle_id) {
        if (type2.empty() || type2[0].empty()) {
            prims = {};
        } else {
            prims = type2[angle_id];
        }
    }

    void makeTwoKNeigh(int k, int Time_resolution, int scale, double agentSize) {
        time_resolution = Time_resolution;
        std::vector<std::pair<int, int>> quad = {{scale, 0}, {0, scale}};

        for (int i = 2; i < k; ++i) {
            std::vector<std::pair<int, int>> newQuad;
            for (auto it1 = quad.begin(); it1 != quad.end(); ++it1) {
                newQuad.push_back(*it1);
                if (std::next(it1) != quad.end()) {
                    int sum_i = it1->first + std::next(it1)->first;
                    int sum_j = it1->second + std::next(it1)->second;
                    newQuad.push_back(std::make_pair(sum_i, sum_j));
                }
            }
            quad = newQuad;
        }

        std::set<std::pair<int, int>> neigh;
        for (auto pair : quad) {
            neigh.insert(pair);
            neigh.insert(std::make_pair(-pair.first, pair.second));
            neigh.insert(std::make_pair(pair.first, -pair.second));
            neigh.insert(std::make_pair(-pair.first, -pair.second));
        }

        type0.push_back({});
        for (auto pair : neigh) {
            Primitive prim;
            prim.id = type0[0].size();
            prim.type = 1;

            prim.duration = sqrt(pair.first * pair.first + pair.second * pair.second) / scale;
            prim.intDuration = prim.doubleToInt(prim.duration, time_resolution);
            prim.agentSize = agentSize * scale;

            prim.source.i = 0;
            prim.source.j = 0;
            prim.source.angle_id = 0;
            prim.source.speed = 0;

            prim.target.i = pair.first;
            prim.target.j = pair.second;
            prim.target.angle_id = 0;
            prim.target.speed = 0;

            prim.i_coefficients.push_back(0);
            prim.i_coefficients.push_back(pair.first / prim.duration);
            prim.i_coefficients.push_back(0);
            prim.i_coefficients.push_back(0);

            prim.j_coefficients.push_back(0);
            prim.j_coefficients.push_back(pair.second / prim.duration);
            prim.j_coefficients.push_back(0);
            prim.j_coefficients.push_back(0);

            prim.countCells();
            prim.countIntervals(0.5, time_resolution);
            type0[0].push_back(prim);
        }
    }
};
#endif
