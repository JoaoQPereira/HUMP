#ifndef HUMP_WAYPOINT_HPP
#define HUMP_WAYPOINT_HPP
#include <string>
#include <vector>

#include "HUMPconfig.hpp"


using namespace std;

namespace HUMotion {

class Waypoint
{
public:


    Waypoint();
    ~Waypoint();
    Waypoint(const Waypoint& wp);
    void setWorkspace(bool &wks);
    void setWaypoint(vector <wp_specs> &wps);
    void setWPparam(vector < wp_specs> &wps, bool &wks );
    bool getWP(vector <wp_specs> &wps);
    void get_wks(bool &wks);
    void setWPnr(int &wp_nr);
    int getWPnr();

private:
    string name;
    vector <wp_specs> waypoints; //  vector of waypoints
    bool workspace;
    int nr_wp;
    const double tf=5;



};
}

#endif
