#ifndef COSTMAP_H
#define COSTMAP_H


#include <fstream>
#include <iostream>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/shared_ptr.hpp>


class CostMap
{
public:
    CostMap(size_t size_x, size_t size_y) 
    : dimx(size_x), dimy(size_y) 
    {
        costmatrix_.resize(dimx, dimy);

        for (unsigned i = 0; i < costmatrix_.size1(); ++i) {
            for (unsigned j = 0; j < costmatrix_.size2(); ++j) {
                costmatrix_(i, j) = 0.0;
            }
        }
    }

    virtual ~CostMap() {
        costmatrix_.clear();
    }

    inline void resizeToOriginal() {
        costmatrix_.resize(dimx, dimy);
    }

    inline void resizeTo(size_t N, size_t M) {
        costmatrix_.resize(N, M);
    }

    /// retrieve using default matrix indexing
    inline double operator() (size_t x, size_t y) {
        assert(x < dimx);
        assert(y < dimy);
        return costmatrix_(x, y);
    }

    /// get the cost map value based on grid coordinates
    // asumes that the grid origin is 0, 0
    inline double getValGrid(double x, double y, double cellwidth, double cellheight) {
        int i = (int)(x / cellwidth);
        int j = (int)(y / cellheight);
        
        assert(i < dimx);
        assert(j < dimy);

        return costmatrix_(i, j);
    }


    /// set a value on the costmap using the matrix indexing
    inline void setVal(int x, int y, double value) {
        assert(x < dimx);
        assert(y < dimy);

        costmatrix_(x, y) = value;
    }


    /// set a value on the costmap using the grid representation
    inline void setValGrid(double x, double y, double cellwidth, double cellheight, double value) {
        int i = (int)(x / cellwidth);
        int j = (int)(y / cellheight);
        
        assert(i < dimx);
        assert(j < dimy);

        costmatrix_(i, j) = value;
    }



    // write the costmap to a file for debugging
    inline void writeToFile(std::string filename) {
        std::ofstream fout(filename.c_str(), std::ios::out);
        if (!fout.is_open()) {
            std::cerr << "Cannot create file [" << filename << "] for saving costmap" << std::endl;
        }

        // save the data in matrix form
        for (unsigned i = 0; i < costmatrix_.size1(); ++i) {
            for (unsigned j = 0; j < costmatrix_.size2(); ++j) {
                fout << costmatrix_(i, j) << " ";
            }
            fout << std::endl;
        }
        fout << std::endl;
        fout.close();

    }

    inline void clear() {
        costmatrix_.clear();
        costmatrix_.resize(dimx, dimy);
    }


protected:
    size_t dimx, dimy;
    boost::numeric::ublas::matrix<double> costmatrix_;
};


typedef boost::shared_ptr<CostMap> CostMapPtr;
typedef boost::shared_ptr<CostMap const> CostMapConstPtr;

#endif