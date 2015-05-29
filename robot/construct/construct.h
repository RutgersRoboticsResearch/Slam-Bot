#ifndef __CONSTRUCT_H__
#define __CONSTRUCT_H__

#include <armadillo>                // for the transformation matrices

// A construct is a rooted graph

class Link {
  public:
    // creating a box in 3d space
    double width;   // x direction
    double length;  // y direction
    double height;  // z direction 

    // Note! Origin is in the center of the object
    
    // orientation (self.orientation is still 0)
    pose3d_t orientation;
};

class Joint {
  // self-imposed rotation item

  public:
    // dynamic parameters
    double min;
    double max;
    double value;
    arma::mat T;
    
    Joint(double min, double max, double initial_value = 0.0);
};

class Construct { // kind of like the linked list class
  private:
    
  public:
    Construct(void);
    ~Construct(void);

    // Two interfaces:
    // 1) Load via string from either buffer or file
    void loads(std::string description);
    void loadf(std::string filename);

    // 2) Load manually by setting links

};

// create a bunch of elements
// construct_generate_element... (using links
// construct_link(element1, element2)

#endif
