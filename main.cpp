#include <iostream>

#include <openMVG/sfm/sfm.hpp>
#include <openMVG/sfm/sfm_data.hpp>

#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/matching/indMatchDecoratorXY.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "common.h"
#include "synth.h"

using namespace std;
using namespace openMVG;
using namespace openMVG::cameras;

using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::sfm;


int
main()
{
    const Pinhole_Intrinsic camL(IMG_WIDTH, IMG_HEIGHT, FX, CX, CY);
    const Pinhole_Intrinsic camR(IMG_WIDTH, IMG_HEIGHT, FX, CX, CY);

    Mat xL, xR;
    get_two_synth_views(xL, xR);

    const std::pair<size_t, size_t> size_imaL(IMG_WIDTH, IMG_HEIGHT);
    const std::pair<size_t, size_t> size_imaR(IMG_WIDTH, IMG_HEIGHT);
    RelativePose_Info relativePose_info;

    if (!robustRelativePose(&camL, &camR, xL, xR, relativePose_info, size_imaL, size_imaR, 16))
    {
      std::cerr << " /!\\ Robust relative pose estimation failure."
        << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "\nFound an Essential matrix:\n"
      << "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
      << "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
      << std::endl;

    std::cout << std::endl
      << "-- Rotation|Translation matrices: --" << "\n"
      << relativePose_info.relativePose.rotation() << "\n\n"
      << relativePose_info.relativePose.translation() << "\n" << std::endl;

    SfM_Data scene;

    scene.views[0].reset(new View("", 0, 0, 0, IMG_WIDTH, IMG_HEIGHT));
    scene.views[1].reset(new View("", 1, 0, 1, IMG_WIDTH, IMG_HEIGHT));

    const Pose3 pose0 = scene.poses[scene.views[0]->id_pose] = Pose3(Mat3::Identity(), Vec3::Zero());
    const Pose3 pose1 = scene.poses[scene.views[1]->id_pose] = relativePose_info.relativePose;

    const Mat34 P1 = camL.get_projective_equivalent(pose0);
    const Mat34 P2 = camR.get_projective_equivalent(pose1);

    scene.intrinsics[0].reset(new Pinhole_Intrinsic(IMG_WIDTH, IMG_HEIGHT, FX, CX, CY));

    Landmarks & landmarks = scene.structure;
    for (int i = 0; i < xL.cols(); i += 1)
    {
        Vec3 X;
        TriangulateDLT(
            P1, xL.col(i).homogeneous(),
            P2, xR.col(i).homogeneous(),
            &X);

        if (pose0.depth(X) < 0 && pose1.depth(X) < 0)
        {
            /* reject point that is behind the camera */
            continue;
        }

        landmarks[i].obs[scene.views[0]->id_view] = Observation(xL.col(i), i);
        landmarks[i].obs[scene.views[1]->id_view] = Observation(xR.col(i), i);
        landmarks[i].X = X;
    }

    Bundle_Adjustment_Ceres bundle_adjustment_obj;
    bundle_adjustment_obj.Adjust(scene,
      Optimize_Options(
        Intrinsic_Parameter_Type::ADJUST_ALL,
        Extrinsic_Parameter_Type::ADJUST_ALL,
        Structure_Parameter_Type::ADJUST_ALL));

    Save(scene, "orange.ply", ESfM_Data(ALL));
}
