/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/terrain/examples/height_map_examples.h>

//#include "home/leo/catkin_ws/src/towr_grid_map/grid2towr.hpp"
//#include "towr_grid_map/grid2towr.h"
//adicionar o meu pacote no package.xml e CMakelist

namespace towr {


FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) 
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) 
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}


// STAIRS
double
Stairs::GetHeight (double x, double y) 
{
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) 
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) 
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) 
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) 
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) 
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}


// Chimney
double
Chimney::GetHeight (double x, double y) 
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end_)
    z = slope_*(y-y_start_);

  return z;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) 
{
  double dzdy = 0.0;

  if (x_start_<= x && x<= x_end_)
    dzdy = slope_;

  return dzdy;
}


// Chimney LR
double
ChimneyLR::GetHeight (double x, double y) 
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end1_)
    z = slope_*(y-y_start_);

  if (x_end1_<=x && x<=x_end2_)
    z = -slope_*(y+y_start_);

  return z;
}

double
ChimneyLR::GetHeightDerivWrtY (double x, double y) 
{
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_<=x && x<=x_end2_)
    dzdy = -slope_;

  return dzdy;
}

// Terreno do Leo
// GAP
double
MyTerrain::GetHeight (double x, double y) 
{
  double h = 0.0;

  //sin
  if (sin_start_ <= x)
    h = a_*sin(w_*x + pi/2);

  return h;
}

double
MyTerrain::GetHeightDerivWrtX (double x, double y) 
{
  double dhdx = 0.0;

  // sin
  if (sin_start_ <= x)
    dhdx = a_*w_*cos(w_*x);

  return dhdx;
}

double
MyTerrain::GetHeightDerivWrtXX (double x, double y) 
{
  double dzdxx = 0.0;

  // sin
  if (sin_start_ <= x)
    dzdxx = -a_*w_*w_*sin(w_*x);

  return dzdxx;
}


// Terreno do Leo
// Terreno definido pelas funcoes de Gridmap


double GridMapTerrain::GetHeight (double x, double y) 
{
  double h = 0.0;
  h = grid2towr.GetElevation(x,y);
  return h;
}

double GridMapTerrain::GetHeightDerivWrtX (double x, double y) 
{
  double dhdx = 0.0;
  dhdx = grid2towr.GetHeightDerivWrtX2(x,y);
  return dhdx;
}

double GridMapTerrain::GetHeightDerivWrtY (double x, double y) 
{
  double dhdy = 0.0;
  dhdy = grid2towr.GetHeightDerivWrtY2(x,y);
  return dhdy;
}

double GridMapTerrain::GetHeightDerivWrtXX (double x, double y) 
{
  double dhdxx = 0.0;
  dhdxx = grid2towr.GetHeightDerivWrtXX(x,y);
  return dhdxx;
}

double GridMapTerrain::GetHeightDerivWrtYY (double x, double y) 
{
  double dhdyy = 0.0;
  dhdyy = grid2towr.GetHeightDerivWrtYY(x,y);
  return dhdyy;
}

double GridMapTerrain::GetHeightDerivWrtXY (double x, double y) 
{
  double dhdxy = 0.0;
  dhdxy = grid2towr.GetHeightDerivWrtXY(x,y);
  return dhdxy;
}

double GridMapTerrain::GetHeightDerivWrtYX (double x, double y) 
{
  double dhdyx = 0.0;
  dhdyx = grid2towr.GetHeightDerivWrtYX(x,y);
  return dhdyx;
}



} /* namespace towr */
