//
// Created by tim on 25.04.20.
//

#ifndef FTMODULE_MAPPOINT_H
#define FTMODULE_MAPPOINT_H


namespace ftmodule {

//TODO(Tim) This should be moved to where the map class lives

class MapPoint {
public:

  int Id() const { return id_; }
  double *GetPositionDataPointer() { return position_w_.data(); }

private:
  int id_;
  Eigen::Vector3d position_w_; ///< The optimized 3d position. This should only be accessed from the estimation thread!!
};

} // namespace ftmodule

#endif //FTMODULE_MAPPOINT_H
