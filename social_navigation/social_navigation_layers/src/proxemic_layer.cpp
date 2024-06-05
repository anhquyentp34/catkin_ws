#include <social_navigation_layers/proxemic_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProxemicLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx), 
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var){
    return sqrt(-2*var * log(cutoff/A) );
    //radius = sqrt(-2*.2025*log(50/255)) = 0.8123
    //radius = sqrt(-2*.2025*log(40/255)) = 0.8661 // default
    //radius = sqrt(-2*.2025*log(30/255)) = 0.9310
    //radius = sqrt(-2*.2025*log(20/255)) =  1.0154
    //radius = sqrt(-2*.2025*log(10/255)) = 1.1453
    //radius = sqrt(-2*.2025*log(1/255)) = 1.4981
    //radius = sqrt(-2*.2025*log(.1/255)) = 1.7823
    //radius = sqrt(-2*.2025*log(.1/255)) = 2.0271

}

//
visualization_msgs::Marker createMarkerPoint(float_t x, float_t y, float_t z, int id_point, int id_zone)
{
    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "dsz_" << id_zone;
    //
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.02;

    marker.color.a = 0.9;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    return marker;

}
//

namespace social_navigation_layers
{
    void ProxemicLayer::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<ProxemicLayerConfig>(nh);
        f_ = boost::bind(&ProxemicLayer::configure, this, _1, _2);
        server_->setCallback(f_);

        dsz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/social_navigation_layers/dsz",0);
    }
    
    void ProxemicLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<people_msgs::Person>::iterator p_it;
        
        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            people_msgs::Person person = *p_it;
             
            double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));

            double factor = 1;
            if(person.reliability)
                factor = 1.0 + mag * factor_;
            else// sitting
                factor = 1.0 + 2.5*mag * factor_;

            double point = get_radius(cutoff_, amplitude_, covar_ * factor );
              
            *min_x = std::min(*min_x, person.position.x - point);
            *min_y = std::min(*min_y, person.position.y - point);
            *max_x = std::max(*max_x, person.position.x + point);
            *max_y = std::max(*max_y, person.position.y + point);
              
        }
    }
    
    void ProxemicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( people_list_.people.size() == 0 )
          return;
        if( cutoff_ >= amplitude_)
            return;
        
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray dsz_marker;
        std::list<people_msgs::Person>::iterator p_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution(); // get the resolution of the costmap
        // process each person
        int id_zone = 0;
        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            id_zone++; // number of dsz
            people_msgs::Person person = *p_it;
            double angle = atan2(person.velocity.y, person.velocity.x)-M_PI_2l;// need -pi/2 like in matlab
            //ROS_INFO("Angle_people%d=%f",id_zone, angle);
            double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));
            double factor = 1;
            if(person.reliability)
                factor = 1.0 + mag * factor_;
            else// sitting
                factor = 1.0 + 2.5*mag * factor_;
            double base = get_radius(cutoff_, amplitude_, covar_); // the radius of the basic personal space
            double point = get_radius(cutoff_, amplitude_, covar_ * factor ); // the radius of the extended personal space
            //base = 4; point = 4;
            unsigned int width = std::max(1, int( (base + point) / res )),
                         height = std::max(1, int( (base + point) / res ));
                          
            double cx = person.position.x, cy = person.position.y;

            double ox, oy;
            if(sin(angle)>0)
                oy = cy - base;
            else
                oy = cy + (point-base) * sin(angle) - base;

            if(cos(angle)>=0)
                ox = cx - base;
            else
                ox = cx + (point-base) * cos(angle) - base;


            int dx, dy;
            costmap->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y=0, end_x=width, end_y = height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap->getSizeInCellsX())
                end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx); //The x size of the costmap

            if((int)(start_x+dx) < min_i)
                start_x = min_i - dx;
            if((int)(end_x+dx) > max_i)
                end_x = max_i - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap->getSizeInCellsY())
                end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

            if((int)(start_y+dy) < min_j)
                start_y = min_j - dy;
            if((int)(end_y+dy) > max_j)
                end_y = max_j - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
            int id_point = 0;
            //ROS_INFO("LocalMapInfo: startx=%d,endx=%d,starty=%d,endy=%d,dx=%d,dy=%d",start_x,end_x,start_y,end_y,dx,dy);
            for(int i=start_x;i<end_x;i++){
                for(int j=start_y;j<end_y;j++){
                  unsigned char old_cost = costmap->getCost(i+dx, j+dy); // get old costmap values
                  if(old_cost == costmap_2d::NO_INFORMATION)
                    continue;

                  double x = bx+i*res, y = by+j*res;
                  double ma = atan2(y-cy,x-cx);
                  double diff = angles::shortest_angular_distance(angle, ma);
                  double a;
                  if(fabs(diff)<M_PI/2)
                      a = gaussian(x,y,cx,cy,amplitude_,covar_*factor,covar_,angle);
                  else
                      a = gaussian(x,y,cx,cy,amplitude_,covar_,       covar_,0);

                  if(a < cutoff_)
                    continue;
                  unsigned char cvalue = (unsigned char) a;
                  costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));// get the max value of the costmaps
                  // use for
                  marker=createMarkerPoint(x,y,a/amplitude_,id_point,id_zone);
                  //marker=createMarkerPoint(x,y,(double)(std::max(cvalue, old_cost))/amplitude_,id_point,id_zone);
                  dsz_marker.markers.push_back(marker);
                  id_point++;
              }
            } 
            dsz_pub_.publish(dsz_marker);
            dsz_marker.markers.clear();
        }
    }

    void ProxemicLayer::configure(ProxemicLayerConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        people_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
    }


};
