//
// Created by dzl on 11/20/25.
//

#include "Preprocess.h"

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(std::shared_ptr<LidarFrame> &msg, PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(PointCloud2::Ptr &msg, PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(std::shared_ptr<PointCloud2> &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    // 1. PointCloud2 → PCL
    PointCloudXYZI::Ptr pl_orig = std::make_shared<PointCloudXYZI>();
    if (!Pointcloud2ToPCL(msg, pl_orig))
        return;

    int plsize = pl_orig->size();
    if (plsize == 0) return;

    pl_full.resize(plsize);
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);

    // -----------------------------
    // feature_enabled = true 分支
    // -----------------------------
    if (feature_enabled)
    {
        // 清空每条 scan 的缓存
        for (int i = 0; i < N_SCANS; i++)
        {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }

        // -------- 第一轮：构建 pl_full & pl_buff ----------
        for (uint i = 1; i < plsize; i++)
        {
            const auto& p = pl_orig->points[i];

            // --- blind filter ---
            double range2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (range2 < blind * blind)
                continue;

            // --------- line & tag 过滤逻辑（与 LivoxFrame 完全一致） --------
            // PointCloud2 不带 line / tag，需要从 reflectivity or ring 额外传输
            // 通常使用 PointField "line" "tag"，和 LivoxPoint 对齐
            int line = 0;
            int tag  = 0;

            // 获取 line / tag （前面你定义的 PointCloud2 → LidarFrame 就支持）
            {
                uint8_t val_line = 0, val_tag = 0;
                // 建议你提前把 line/tag 在 PCL intensity 后面扩展写入 curvature 或 normal_x
                // 这里假设 curvature=offset_time, normal_x=line, normal_y=tag
                line = static_cast<int>(p.normal_x);
                tag  = static_cast<int>(p.normal_y);
            }

            if (line >= N_SCANS) continue;
            if (!((tag & 0x30) == 0x10 || (tag & 0x30) == 0x00))
                continue;

            // ---------- 填充 pl_full[i] ----------
            pl_full[i].x = p.x;
            pl_full[i].y = p.y;
            pl_full[i].z = p.z;
            pl_full[i].intensity = p.intensity;

            // curvature = offset_time (ms)
            pl_full[i].curvature = p.curvature / float(1000000);

            // ---------- 重复点剔除 ----------
            const auto& q = pl_orig->points[i-1];
            if ((fabs(p.x - q.x) > 1e-7) ||
                (fabs(p.y - q.y) > 1e-7) ||
                (fabs(p.z - q.z) > 1e-7))
            {
                pl_buff[line].push_back(pl_full[i]);
            }
        }

        // -------- 第二轮：逐线提取特征 give_feature() ----------
        static int count = 0;
        static double total_time = 0;

        double t0 = omp_get_wtime();

        for (int j = 0; j < N_SCANS; j++)
        {
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            if (pl.size() <= 5) continue;

            uint32_t sz = pl.size();
            std::vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(sz);

            // 计算点间距离
            for (uint i = 0; i + 1 < sz; i++)
            {
                types[i].range =
                    sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);

                float vx = pl[i].x - pl[i+1].x;
                float vy = pl[i].y - pl[i+1].y;
                float vz = pl[i].z - pl[i+1].z;
                types[i].dista = sqrt(vx*vx + vy*vy + vz*vz);
            }

            types[sz - 1].range =
                sqrt(pl[sz-1].x * pl[sz-1].x + pl[sz-1].y * pl[sz-1].y);

            // Fast-LIO 原始特征提取函数
            give_feature(pl, types);
        }

        total_time += (omp_get_wtime() - t0);
        count++;
        printf("Feature extraction time: %lf ms \n", total_time / count * 1000.0);
    }

    // -----------------------------
    // feature_enabled = false 分支
    // -----------------------------
    else
    {
        uint valid_num = 0;

        for (uint i = 1; i < plsize; i++)
        {
            const auto& p = pl_orig->points[i];

            // --- line & tag 过滤 ---
            int line = static_cast<int>(p.normal_x);
            int tag  = static_cast<int>(p.normal_y);
            if (line >= N_SCANS) continue;
            if (!((tag & 0x30) == 0x10 || (tag & 0x30) == 0x00))
                continue;

            // 降采样
            valid_num++;
            if (valid_num % point_filter_num != 0)
                continue;

            // blind
            float range2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (range2 < blind * blind)
                continue;

            PointType q;
            q.x = p.x;
            q.y = p.y;
            q.z = p.z;
            q.intensity = p.intensity;
            q.curvature = p.curvature / float(1000000);

            // 去除重复点（效果更好）
            const auto& last = pl_orig->points[i - 1];
            if ((fabs(p.x - last.x) > 1e-7) ||
                (fabs(p.y - last.y) > 1e-7) ||
                (fabs(p.z - last.z) > 1e-7))
            {
                pl_surf.push_back(q);
            }
        }
    }
}



void Preprocess::avia_handler(std::shared_ptr<LidarFrame> &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      std::vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types) {
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      {
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  std::vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}