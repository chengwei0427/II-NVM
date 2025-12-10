#include "lio/lio_utils.h"

namespace zjloc
{
     state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_,
                  const Eigen::Vector3d &velocity_, const Eigen::Vector3d &ba_,
                  const Eigen::Vector3d &bg_)
         : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
     {
     }

     state::state(const state *state_temp, bool copy)
     {
          if (copy)
          {
               rotation = state_temp->rotation;
               translation = state_temp->translation;

               velocity = state_temp->velocity;
               ba = state_temp->ba;
               bg = state_temp->bg;
          }
          else
          {
               rotation = state_temp->rotation;
               translation = state_temp->translation;

               velocity = state_temp->velocity;
               ba = state_temp->ba;
               bg = state_temp->bg;
          }
     }

     void state::release()
     {
     }

     //   --------------------------------------     //
     cloudFrame::cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_,
                            state *p_state_)
     {
          point_surf.insert(point_surf.end(), point_surf_.begin(), point_surf_.end());
          const_surf.insert(const_surf.end(), const_surf_.begin(), const_surf_.end());

          // p_state = p_state_;
          p_state = new state(p_state_, true);
     }

     cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)
     {
          time_frame_begin = p_cloud_frame->time_frame_begin;
          time_frame_end = p_cloud_frame->time_frame_end;

          frame_id = p_cloud_frame->frame_id;

          p_state = new state(p_cloud_frame->p_state, true);

          point_surf.insert(point_surf.end(), p_cloud_frame->point_surf.begin(), p_cloud_frame->point_surf.end());
          const_surf.insert(const_surf.end(), p_cloud_frame->const_surf.begin(), p_cloud_frame->const_surf.end());
     }

     void cloudFrame::release()
     {

          std::vector<point3D>().swap(point_surf);
          std::vector<point3D>().swap(const_surf);

          p_state = nullptr;
     }
}