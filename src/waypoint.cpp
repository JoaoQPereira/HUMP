#include "../include/waypoint.hpp"

namespace HUMotion{


Waypoint::Waypoint()
{
    this->name="";
    this->waypoints.clear();
    this->workspace=0;
}

Waypoint::Waypoint(const Waypoint &wp)
{
    this->nr_wp = wp.nr_wp;
    this->workspace=wp.workspace;
    this->waypoints = vector<wp_specs>(wp.waypoints.size());
    std::copy(wp.waypoints.begin(),wp.waypoints.end(),this->waypoints.begin());
}

Waypoint:: ~Waypoint()
{

}


void Waypoint::setWorkspace(bool &wks){
    this->workspace = wks;
}

void Waypoint::setWaypoint(vector <wp_specs> &wps)
{
    for (int i=0;i<wps.size();i++){
        this->waypoints.push_back(wps[i]);
    }
   // this->waypoints = vector <wp_specs> (wps.size());
   // std::copy(wps.begin(),wps.end(),this->waypoints.begin());

}

void Waypoint::setWPparam(vector <wp_specs> &wps, bool &wks )
{
    this->setWaypoint(wps);
    this->setWorkspace(wks);
}


bool Waypoint::getWP(vector <wp_specs> &wps)
{
    wps.clear();

    if(!this->waypoints.empty())
    {
        wps = std::vector<wp_specs>(this->waypoints.size());
        std::copy(this->waypoints.begin(),this->waypoints.end(),wps.begin());
        return true;
    }
    else
        return false;

}

void Waypoint::get_wks(bool &wp_wks){
    wp_wks=this->workspace;
}

void Waypoint::setWPnr(int &wp_nr){
    this->nr_wp=wp_nr;
}

int Waypoint::getWPnr()
{
    return this->nr_wp;

}


}
