
#if 0

/*
 *
#include <mutex>
#include <boost/thread/thread.hpp>

static std::mutex mutex;
static int count = 0;

void Counter() {
  std::unique_lock<std::mutex> lock(mutex);
  int i = ++count;
  std::cout << "count == " << i << std::endl;
}


int main() {
  boost::thread_group threads;
  for (int i = 0; i < 4; ++i) {
    threads.create_thread(&Counter);
  }

  threads.join_all();
  return 0;
}
*/


///设置Anchor Map
typedef struct ATPosition{
  double x;
  double y;
  double z;
}ATPosition;


static std::map<uint,ATPosition>  anchor_map;
//auto anchor = anchor_map.at( 000 );
//printf("anchor %d : %f %f %f \n", 0, anchor.x , anchor.y, anchor.z );
//单位m
static double get_distance_from_position( const std::map<uint,ATPosition> anchormap,
                                        const uint anchorid,
                                        const ATPosition tag_p ){
    auto anchor_p = anchormap.at( anchorid );
    double ds = std::pow( anchor_p.x - tag_p.x , 2 ) +
              std::pow( anchor_p.y - tag_p.y , 2 ) +
              std::pow( anchor_p.z - tag_p.z , 2 );
    return std::sqrt( ds );
};


//anchor_map.insert(std::pair<uint,ATPosition>(12377,{1.60, 0.78, 3.44}) );
//anchor_map.insert(std::pair<uint,ATPosition>(12401,{8.46, 1.04, 3.45}) );
//anchor_map.insert(std::pair<uint,ATPosition>(12349,{1.59, 4.47, 3.44}) );
//anchor_map.insert(std::pair<uint,ATPosition>(12382,{8.46, 5.03, 3.44}) );
        /*
        ATPosition tag_p;
        tag_p.x =  p->px / 100;
        tag_p.y =  p->py / 100;
        tag_p.z =  p->pz / 100;
        double d1 = get_distance_from_position( anchor_map, 12377, tag_p );
        double d2 = get_distance_from_position( anchor_map, 12401, tag_p );
        double d3 = get_distance_from_position( anchor_map, 12349, tag_p );
        double d4 = get_distance_from_position( anchor_map, 12382, tag_p );
        printf("%d: distance: %f %f %f %f m\n", ++d_count, d1,d2, d3, d4 );
        if( pd_count == 0 )
            pub_tag_distance( tag_distance_publisher , 12377 , uint( d1 * 1000 ), 50 );
        else if( pd_count == 1 )
            pub_tag_distance( tag_distance_publisher , 12401 , uint( d2 * 1000 ), 50 );
        else if( pd_count == 2 )
            pub_tag_distance( tag_distance_publisher , 12349 , uint( d3 * 1000 ), 50 );
        else if( pd_count == 3 )
            pub_tag_distance( tag_distance_publisher , 12382 , uint( d4 * 1000 ), 50 );
        pd_count++;
        if( pd_count >= 4 )
          pd_count = 0;
        */
#endif

