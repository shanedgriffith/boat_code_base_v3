#include <stdio.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>    //calibration and performs projections
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/slam/EssentialMatrixFactor.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h> //for calculating the marginal covariance of the desired variables.
#include <gtsam/nonlinear/Values.h>//the initial guess of each thing (be it a pose3, point3, point3, whatever).
#include <gtsam/nonlinear/Symbol.h>//using symbols to identify factors
#include <gtsam/base/debug.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>  //used for visual slam. this is one nonlinear optimizer.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

#include "DataTypes/Camera.hpp"
#include "BoatSurvey/ParseBoatSurvey.hpp"
#include "FileParsing/FileParsing.hpp"

#include "testE.h"

//active set from camera pose 110 of 140106.
std::vector<gtsam::Point2> p2d0 = {gtsam::Point2(596.55, 168.12), gtsam::Point2(492.77, 163.2), gtsam::Point2(607.79, 196.97), gtsam::Point2(601.18, 146.02), gtsam::Point2(571.15, 184.86), gtsam::Point2(604.3, 187.3), gtsam::Point2(601.74, 173.42), gtsam::Point2(535.66, 207.84), gtsam::Point2(410.29, 151.57), gtsam::Point2(409.15, 177.71), gtsam::Point2(469.31, 209.2), gtsam::Point2(536.17, 210.47), gtsam::Point2(474.81, 142.5), gtsam::Point2(628.67, 333.46), gtsam::Point2(623.68, 301.9), gtsam::Point2(635.59, 334.02), gtsam::Point2(601.21, 349.35), gtsam::Point2(487.31, 81.88), gtsam::Point2(461.19, 94.63), gtsam::Point2(574.48, 174.82), gtsam::Point2(578.22, 177.9), gtsam::Point2(582.65, 190.64), gtsam::Point2(385.44, 182.98), gtsam::Point2(376.07, 62.98), gtsam::Point2(598.11, 296.71), gtsam::Point2(389.01, 102.63), gtsam::Point2(392.83, 39.84), gtsam::Point2(407.89, 94.29), gtsam::Point2(365.61, 120.03), gtsam::Point2(460.55, 7.34), gtsam::Point2(429.37, 14.01), gtsam::Point2(595.43, 341.09), gtsam::Point2(609.96, 304.23), gtsam::Point2(625.73, 349.4), gtsam::Point2(364.1, 128.31), gtsam::Point2(361.86, 27.99), gtsam::Point2(571.88, 159.64), gtsam::Point2(442.43, 54.81), gtsam::Point2(448.99, 71.77), gtsam::Point2(550.02, 324.04), gtsam::Point2(444.34, 2.94), gtsam::Point2(351.39, 49.5), gtsam::Point2(355.99, 11.46), gtsam::Point2(319.95, 236.57), gtsam::Point2(454.84, 26.47), gtsam::Point2(367.4, 4.25), gtsam::Point2(618.35, 152.04), gtsam::Point2(639.73, 200.78), gtsam::Point2(542.59, 163.77), gtsam::Point2(350.43, 227.92), gtsam::Point2(675.33, 177), gtsam::Point2(367.63, 88.83), gtsam::Point2(472.47, 123.26), gtsam::Point2(350.6, 227.34), gtsam::Point2(598.38, 105.03), gtsam::Point2(626.6, 355.53), gtsam::Point2(684.34, 142.39), gtsam::Point2(389.45, 90.32), gtsam::Point2(350.96, 219.2), gtsam::Point2(351.21, 227.95), gtsam::Point2(598.06, 97.03), gtsam::Point2(317.51, 140.62), gtsam::Point2(501.78, 67.13), gtsam::Point2(366.99, 142.25), gtsam::Point2(332.5, 68.25), gtsam::Point2(394.78, 104.56), gtsam::Point2(309.02, 174.27), gtsam::Point2(449.25, 48.26), gtsam::Point2(469.9, 57.91), gtsam::Point2(493.29, 204.41), gtsam::Point2(280.28, 316.09), gtsam::Point2(339.28, 329.17), gtsam::Point2(365.26, 149.33), gtsam::Point2(165.78, 240.38), gtsam::Point2(390.01, 232.23), gtsam::Point2(222.55, 105.89), gtsam::Point2(287.99, 189.19), gtsam::Point2(403.27, 292.61), gtsam::Point2(468.66, 267.5), gtsam::Point2(206.54, 65.69), gtsam::Point2(594.42, 241.64), gtsam::Point2(206.89, 144.63), gtsam::Point2(194.56, 165.36), gtsam::Point2(223.48, 136.64), gtsam::Point2(408.91, 23.81), gtsam::Point2(242.63, 25.9), gtsam::Point2(170.15, 186.8), gtsam::Point2(460.86, 226.8), gtsam::Point2(254.95, 132.08), gtsam::Point2(350.68, 99.99), gtsam::Point2(542.17, 243.97), gtsam::Point2(462.45, 262.68), gtsam::Point2(465.28, 271.4), gtsam::Point2(192.01, 166.51), gtsam::Point2(181.99, 162.44), gtsam::Point2(171.84, 143.23), gtsam::Point2(454.93, 211.95), gtsam::Point2(437.16, 205.16), gtsam::Point2(156.89, 230.26), gtsam::Point2(129.34, 216.95), gtsam::Point2(136.88, 250.88), gtsam::Point2(131.14, 241.34), gtsam::Point2(212.28, 28.87), gtsam::Point2(190.17, 23.34), gtsam::Point2(250.1, 159.22), gtsam::Point2(674.86, 191.84), gtsam::Point2(444.57, 106.39), gtsam::Point2(464.1, 127.45), gtsam::Point2(324.76, 60.4), gtsam::Point2(369.58, 297.99), gtsam::Point2(515.36, 173.91), gtsam::Point2(537.08, 161.57), gtsam::Point2(174.91, 172.32), gtsam::Point2(138.83, 175.51), gtsam::Point2(175.67, 75.24), gtsam::Point2(192.8, 99.1), gtsam::Point2(177.7, 76.35), gtsam::Point2(191.26, 86.71), gtsam::Point2(185.09, 94.47), gtsam::Point2(368.4, 259.08), gtsam::Point2(165.65, 111.61), gtsam::Point2(157.57, 67.96), gtsam::Point2(77.83, 278.92), gtsam::Point2(99.09, 253.71), gtsam::Point2(75.2, 278.05), gtsam::Point2(527.47, 200.36), gtsam::Point2(507.13, 197.6), gtsam::Point2(133.09, 178.81), gtsam::Point2(182.37, 26.03), gtsam::Point2(167.65, 17.48), gtsam::Point2(119.91, 142.26), gtsam::Point2(119.6, 115.44), gtsam::Point2(76.86, 304.83), gtsam::Point2(508.27, 235.82), gtsam::Point2(499.24, 249.12), gtsam::Point2(126.62, 95.01), gtsam::Point2(335.34, 78.44), gtsam::Point2(104.14, 157.74), gtsam::Point2(113.13, 174.87), gtsam::Point2(109.46, 87.14), gtsam::Point2(117.55, 99.49), gtsam::Point2(407.08, 112.34), gtsam::Point2(272.14, 186.48), gtsam::Point2(279.04, 183.39), gtsam::Point2(144.73, 30.41), gtsam::Point2(393.09, 61.56), gtsam::Point2(59.28, 217.67), gtsam::Point2(439.13, 124.56), gtsam::Point2(453.36, 144.05), gtsam::Point2(62.34, 218.77), gtsam::Point2(104.25, 196.85), gtsam::Point2(335.75, 26.69), gtsam::Point2(113.11, 118.65), gtsam::Point2(84.45, 125.54), gtsam::Point2(121.36, 19.48), gtsam::Point2(349.77, 167.72), gtsam::Point2(335.57, 173.68), gtsam::Point2(281.87, 53.62), gtsam::Point2(302.72, 39.66), gtsam::Point2(137.52, 293.2), gtsam::Point2(94.88, 80.25), gtsam::Point2(90.36, 245.32), gtsam::Point2(552.68, 249.69), gtsam::Point2(58.67, 309.75), gtsam::Point2(164.14, 255.17), gtsam::Point2(137.47, 289.18), gtsam::Point2(33.19, 274.79), gtsam::Point2(37.89, 254.1), gtsam::Point2(403.17, 297.44), gtsam::Point2(99.9, 54.98), gtsam::Point2(118.04, 69.6), gtsam::Point2(483.82, 273.47), gtsam::Point2(171.8, 193.95), gtsam::Point2(184.99, 199.38), gtsam::Point2(279.36, 154.29), gtsam::Point2(55.33, 173), gtsam::Point2(62.57, 147.89), gtsam::Point2(80.05, 119.42), gtsam::Point2(263.29, 151.39), gtsam::Point2(278.04, 175.88), gtsam::Point2(248.98, 163.38), gtsam::Point2(310.16, 261.64), gtsam::Point2(72.04, 76.12), gtsam::Point2(313.73, 202), gtsam::Point2(287.82, 30.05), gtsam::Point2(303.32, 35.58), gtsam::Point2(48.82, 144.94), gtsam::Point2(280.87, 104.56), gtsam::Point2(310.43, 94), gtsam::Point2(26.42, 303.77), gtsam::Point2(106.08, 269.24), gtsam::Point2(112.14, 280.08), gtsam::Point2(220.82, 329.78), gtsam::Point2(39.89, 207.22), gtsam::Point2(22.79, 191.27), gtsam::Point2(47.5, 191.57), gtsam::Point2(256.51, 191.25), gtsam::Point2(235.31, 217.34), gtsam::Point2(85.79, 177.26), gtsam::Point2(151.55, 102.27), gtsam::Point2(131.01, 95.55), gtsam::Point2(519.4, 259.79), gtsam::Point2(517.44, 280.78), gtsam::Point2(229.41, 299.16), gtsam::Point2(538.87, 283.16), gtsam::Point2(204.13, 220.06), gtsam::Point2(183.55, 204.58), gtsam::Point2(208.08, 202.7), gtsam::Point2(595.85, 273.44), gtsam::Point2(347.59, 270.52), gtsam::Point2(403.85, 45.04), gtsam::Point2(65.39, 227.24), gtsam::Point2(70.85, 245.31), gtsam::Point2(237.87, 284.65), gtsam::Point2(221.47, 279.77), gtsam::Point2(44, 280), gtsam::Point2(63, 278), gtsam::Point2(429, 430), gtsam::Point2(238, 231), gtsam::Point2(219, 244), gtsam::Point2(228, 180), gtsam::Point2(236, 155), gtsam::Point2(258, 97)};
std::vector<gtsam::Point2> p2d1 = {gtsam::Point2(607.67, 166.08), gtsam::Point2(504.94, 162.52), gtsam::Point2(629.1, 196.05), gtsam::Point2(610.85, 144.69), gtsam::Point2(583.21, 183.68), gtsam::Point2(616.2, 185.55), gtsam::Point2(613.6, 171.89), gtsam::Point2(547.46, 206.9), gtsam::Point2(439.83, 150.69), gtsam::Point2(422.78, 179.48), gtsam::Point2(480.47, 208.42), gtsam::Point2(548.01, 209.45), gtsam::Point2(498.25, 143.55), gtsam::Point2(661.1, 332.31), gtsam::Point2(655.65, 299.69), gtsam::Point2(665.71, 333.9), gtsam::Point2(632.58, 348.52), gtsam::Point2(517.68, 80.21), gtsam::Point2(489.92, 92.87), gtsam::Point2(586.85, 173.21), gtsam::Point2(590.07, 176.44), gtsam::Point2(595.33, 188.94), gtsam::Point2(407.49, 181.99), gtsam::Point2(405.4, 60.81), gtsam::Point2(630.59, 296.08), gtsam::Point2(418.62, 101.07), gtsam::Point2(424.48, 37.55), gtsam::Point2(436.81, 93.24), gtsam::Point2(395.17, 118.87), gtsam::Point2(497.01, 5), gtsam::Point2(458.95, 12.53), gtsam::Point2(625.84, 339.29), gtsam::Point2(640.75, 300.48), gtsam::Point2(657.7, 348.47), gtsam::Point2(393.95, 126.71), gtsam::Point2(391.62, 25.55), gtsam::Point2(582.9, 158.36), gtsam::Point2(474.79, 50.99), gtsam::Point2(478.55, 70.34), gtsam::Point2(579.27, 321.87), gtsam::Point2(475.57, 1.54), gtsam::Point2(379.28, 52.9), gtsam::Point2(387.21, 14.63), gtsam::Point2(351.49, 236.23), gtsam::Point2(487.02, 25.18), gtsam::Point2(397.75, 5.55), gtsam::Point2(627.68, 151), gtsam::Point2(671.15, 197.8), gtsam::Point2(554.28, 162.42), gtsam::Point2(380.33, 224.14), gtsam::Point2(689.52, 175.26), gtsam::Point2(397.26, 87.54), gtsam::Point2(501.18, 121.27), gtsam::Point2(380.7, 223.69), gtsam::Point2(606.98, 103.67), gtsam::Point2(658.07, 354.14), gtsam::Point2(696.95, 140.78), gtsam::Point2(422.71, 88.07), gtsam::Point2(377.73, 219.7), gtsam::Point2(380.92, 223.82), gtsam::Point2(606.76, 95.41), gtsam::Point2(350.94, 142.37), gtsam::Point2(533, 65.03), gtsam::Point2(397.2, 140.6), gtsam::Point2(366.14, 70.07), gtsam::Point2(424.48, 102.96), gtsam::Point2(343.66, 171.96), gtsam::Point2(481.01, 46.14), gtsam::Point2(500.76, 56.17), gtsam::Point2(505, 203.54), gtsam::Point2(311.41, 314.5), gtsam::Point2(369.84, 327.84), gtsam::Point2(395.42, 147.88), gtsam::Point2(192.03, 239.36), gtsam::Point2(410.12, 230.57), gtsam::Point2(256.56, 107.54), gtsam::Point2(314.65, 188.13), gtsam::Point2(414.03, 290.64), gtsam::Point2(479.72, 267.17), gtsam::Point2(240.71, 62.13), gtsam::Point2(611.06, 239.2), gtsam::Point2(237.67, 144.03), gtsam::Point2(223.25, 163.54), gtsam::Point2(257.75, 135.17), gtsam::Point2(442.76, 20.91), gtsam::Point2(276.8, 24.38), gtsam::Point2(193.6, 182.77), gtsam::Point2(473.45, 227.58), gtsam::Point2(286.41, 131.78), gtsam::Point2(379, 97.24), gtsam::Point2(554.59, 242.44), gtsam::Point2(473.86, 262.16), gtsam::Point2(475.11, 270.36), gtsam::Point2(220.86, 163.7), gtsam::Point2(214.51, 160.01), gtsam::Point2(199.17, 142.16), gtsam::Point2(466.22, 211.05), gtsam::Point2(454, 203.84), gtsam::Point2(179.43, 230.35), gtsam::Point2(148.4, 216.47), gtsam::Point2(162.41, 248.84), gtsam::Point2(149.48, 241.63), gtsam::Point2(246.07, 29.14), gtsam::Point2(222.44, 23.29), gtsam::Point2(282.3, 157.71), gtsam::Point2(689.23, 190.12), gtsam::Point2(472.91, 104.46), gtsam::Point2(493.13, 126.13), gtsam::Point2(358.19, 63.13), gtsam::Point2(380.78, 295.66), gtsam::Point2(527.19, 173.11), gtsam::Point2(548.78, 160.17), gtsam::Point2(195.46, 167.63), gtsam::Point2(167.14, 173.5), gtsam::Point2(209.73, 75.63), gtsam::Point2(222.8, 95.85), gtsam::Point2(212.91, 77.91), gtsam::Point2(220.97, 85.94), gtsam::Point2(218.54, 92.4), gtsam::Point2(381.51, 264.94), gtsam::Point2(197.51, 111.08), gtsam::Point2(192.67, 69.04), gtsam::Point2(92.61, 278.73), gtsam::Point2(128.35, 252.57), gtsam::Point2(91.29, 277.39), gtsam::Point2(539.33, 199.26), gtsam::Point2(518.14, 196.55), gtsam::Point2(162.76, 177), gtsam::Point2(213.61, 25.37), gtsam::Point2(202.12, 16.89), gtsam::Point2(152.77, 141.4), gtsam::Point2(155.48, 114.87), gtsam::Point2(89.8, 303.37), gtsam::Point2(518.53, 234.69), gtsam::Point2(509.49, 248.23), gtsam::Point2(162.05, 94.21), gtsam::Point2(366.23, 80), gtsam::Point2(138.02, 156.73), gtsam::Point2(146.97, 174.04), gtsam::Point2(146.71, 86.45), gtsam::Point2(152.6, 99.66), gtsam::Point2(435.35, 110.98), gtsam::Point2(303.09, 186.93), gtsam::Point2(310.57, 184.87), gtsam::Point2(183.06, 29.71), gtsam::Point2(422.15, 60.26), gtsam::Point2(87.13, 216.35), gtsam::Point2(466.5, 123.76), gtsam::Point2(478.77, 143.35), gtsam::Point2(89.9, 217.3), gtsam::Point2(136.08, 195.6), gtsam::Point2(372.08, 25.03), gtsam::Point2(147.59, 117.58), gtsam::Point2(119.57, 124.84), gtsam::Point2(161.25, 19.61), gtsam::Point2(365.55, 166.96), gtsam::Point2(358.35, 173.66), gtsam::Point2(313.14, 53.09), gtsam::Point2(331.55, 39.73), gtsam::Point2(149.41, 293.69), gtsam::Point2(133.03, 79.38), gtsam::Point2(119.75, 244.05), gtsam::Point2(564.33, 247.65), gtsam::Point2(72.23, 308.16), gtsam::Point2(192.02, 253.56), gtsam::Point2(149.06, 289.21), gtsam::Point2(48.09, 275.88), gtsam::Point2(57.58, 253.36), gtsam::Point2(414.16, 297.19), gtsam::Point2(140.25, 55.08), gtsam::Point2(154.84, 68.21), gtsam::Point2(493.32, 273.22), gtsam::Point2(198.39, 190.66), gtsam::Point2(202.82, 195.18), gtsam::Point2(310.22, 154.69), gtsam::Point2(90.28, 171.83), gtsam::Point2(98.35, 147.11), gtsam::Point2(117.99, 118.43), gtsam::Point2(290.65, 150.39), gtsam::Point2(310.34, 171.86), gtsam::Point2(281.44, 162.6), gtsam::Point2(343.31, 264.51), gtsam::Point2(110.95, 75.8), gtsam::Point2(345.65, 200.97), gtsam::Point2(319.92, 29.96), gtsam::Point2(331.87, 35.93), gtsam::Point2(84.08, 144.15), gtsam::Point2(311.62, 105.51), gtsam::Point2(347.96, 92.18), gtsam::Point2(38.7, 301.66), gtsam::Point2(124.22, 269.88), gtsam::Point2(124.7, 279.71), gtsam::Point2(250.19, 327.9), gtsam::Point2(67.79, 205.75), gtsam::Point2(51.51, 191.24), gtsam::Point2(74.72, 189.56), gtsam::Point2(284.2, 188.04), gtsam::Point2(262.18, 215.2), gtsam::Point2(121.29, 176.13), gtsam::Point2(183.11, 100.59), gtsam::Point2(164.78, 95.57), gtsam::Point2(529.53, 258.76), gtsam::Point2(527.69, 279.71), gtsam::Point2(244.04, 300.23), gtsam::Point2(550.66, 282.4), gtsam::Point2(217.08, 226.43), gtsam::Point2(203, 207.44), gtsam::Point2(223.42, 197.13), gtsam::Point2(620.22, 270.99), gtsam::Point2(375.1, 269.98), gtsam::Point2(435.47, 43.52), gtsam::Point2(90.45, 224.37), gtsam::Point2(102.29, 244.41), gtsam::Point2(250.53, 280.06), gtsam::Point2(242.95, 277.75), gtsam::Point2(66.84, 280.28), gtsam::Point2(78.67, 278), gtsam::Point2(440.82, 421.78), gtsam::Point2(262.74, 217.85), gtsam::Point2(230.68, 243.58), gtsam::Point2(256.4, 177.24), gtsam::Point2(269.02, 152.96), gtsam::Point2(292.24, 98.46)};


void
TestE::
verifyOptimizationForE2()
{
    //the noise model of 0.01 seemed close, but flipped in translation. the noise model of 1.0 seemed spot-on, but negated in rotation and in translation.
    
    //Create two cameras and corresponding essential matrix E
    gtsam::Pose3 pose_t2(gtsam::Rot3::Expmap(gtsam::Vector3(-1.251, -1.1194, 1.073)), gtsam::Point3(7.3938, -35.725, -0.061478));
    gtsam::Pose3 pose_t1(gtsam::Rot3::Expmap(gtsam::Vector3(-1.2479, -1.1258, 1.0751)), gtsam::Point3(7.2857, -36.169, -0.063827));
    Camera axisptz = ParseBoatSurvey::GetCamera();
    gtsam::Cal3_S2::shared_ptr cam = axisptz.GetGTSAMCam();
    
    //We start with a factor graph and add epipolar constraints to it
    //Noise sigma is 1cm, assuming metric measurements
    gtsam::NonlinearFactorGraph graph;
    auto model = gtsam::noiseModel::Isotropic::Sigma(1, 1); //1D?
    
    for(int i=0; i<p2d0.size(); ++i)
    {
        gtsam::Point2 p0 = cam->calibrate(p2d0[i]);
        gtsam::Point2 p1 = cam->calibrate(p2d1[i]);
        graph.add(gtsam::EssentialMatrixFactor(1, p0, p1, model));
    }
    
    gtsam::Pose3 btwn = pose_t2.between(pose_t1);
    gtsam::Unit3 aTb(btwn.translation());
    //Create initial estimate
    gtsam::EssentialMatrix trueE(btwn.rotation(), aTb);
    gtsam::Values initial;
    initial.insert(1, trueE);
    
    //Optimize
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    std::cout << "optimization running..." << std::endl;
    gtsam::Values result = optimizer.optimize();
    std::cout << "optimization finished..." << std::endl;
    
    //Print result (as essentialMatrix) and error
    double error = graph.error(result);
    gtsam::EssentialMatrix E = result.at<gtsam::EssentialMatrix>(1);
    gtsam::Rot3 tr = E.rotation().inverse();
    gtsam::Rot3 pix = gtsam::Rot3::Rx(3.14159);
    
//    std::cout << "essential matrix: \n" << E.direction().unitVector().transpose() << ", " << gtsam::Rot3::Logmap(E.rotation()).transpose() << std::endl;
    std::cout << "essential matrix: \n" << pix * (E.direction().unitVector()).transpose() << ", " << gtsam::Rot3::Logmap(E.rotation()).transpose() << std::endl;
//    std::cout << "essential matrix: \n" << E.rotation().inverse() * (E.direction().unitVector()).transpose() << ", " << gtsam::Rot3::Logmap(E.rotation().inverse()).transpose() << std::endl;
//    std::cout << "expected: \n" << trueE.direction().unitVector().transpose() << ", " << gtsam::Rot3::Logmap(trueE.rotation()).transpose() << std::endl;
    
    gtsam::EssentialMatrix le = gtsam::EssentialMatrix::FromPose3(gtsam::Pose3(E.rotation(), pix * (E.direction().unitVector())));
    
    gtsam::Symbol s0('x', 0);
    gtsam::Symbol s1('x', 1);
    gtsam::Vector5 v5p;
    v5p = (gtsam::Vector(5) << 1, 1, 1, 1, 1).finished();
    auto noisemodel = gtsam::noiseModel::Diagonal::Sigmas(v5p);
    
    gtsam::EssentialMatrixConstraint emc(s0, s1, le, noisemodel);
    gtsam::Vector err = emc.evaluateError(pose_t2, pose_t1);
    std::cout << "E error: " << err.transpose() << ", total : " << err.norm() << std::endl;
    
    emc = gtsam::EssentialMatrixConstraint(s0, s1, E, noisemodel);
    err = emc.evaluateError(pose_t2, pose_t1);
    std::cout << "E error: " << err.transpose() << ", total : " << err.norm() << std::endl;
    
}


void
TestE::
verifyOptimizationForE()
{
    //Create two cameras and corresponding essential matrix E
    gtsam::Rot3 aRb = gtsam::Rot3::Yaw(3.1415926/2.0);
    gtsam::Point3 aTb(0.1, 0, 0);
    gtsam::Pose3 identity;
    gtsam::Pose3 aPb(aRb, aTb);
    gtsam::CalibratedCamera cameraA(identity);
    gtsam::CalibratedCamera cameraB(aPb);
    
    //Create 5 points
    std::vector<gtsam::Point3> P =
    {
        gtsam::Point3(0, 0, 1),
        gtsam::Point3(-0.1, 0, 1),
        gtsam::Point3(0.1, 0, 1),
        gtsam::Point3(0, 0.5, 0.5),
        gtsam::Point3(0, -0.5, 0.5)
    };
    
    //Project points in both cameras
    std::vector<gtsam::Point2> pA(5), pB(5);
    for(int i=0; i<5; ++i)
    {
        pA[i] = cameraA.project(P[i]);
        pB[i] = cameraB.project(P[i]);
    }
    
    //We start with a factor graph and add epipolar constraints to it
    //Noise sigma is 1cm, assuming metric measurements
    gtsam::NonlinearFactorGraph graph;
    auto model = gtsam::noiseModel::Isotropic::Sigma(1, 0.01); //1D?
    for(int i=0; i<5; ++i)
    {
        graph.add(gtsam::EssentialMatrixFactor(1, pA[i], pB[i], model));
    }
    
    //Create initial estimate
    gtsam::EssentialMatrix trueE = gtsam::EssentialMatrix::FromPose3(gtsam::Pose3(aRb, aTb));
    gtsam::Vector5 noise;
    noise = (gtsam::Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1).finished();
    gtsam::EssentialMatrix initialE = trueE.retract(noise);
    gtsam::Values initial;
    initial.insert(1, initialE);
    
    //Optimize
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();
    
    //Print result (as essentialMatrix) and error
    double error = graph.error(result);
    gtsam::EssentialMatrix E = result.at<gtsam::EssentialMatrix>(1);
    
    std::cout << "essential matrix: \n" << E << std::endl;
    std::cout << "expected: \n" << trueE << std::endl;
    
    gtsam::Symbol s0('x', 0);
    gtsam::Symbol s1('x', 1);
    gtsam::Vector5 v5p;
    v5p = (gtsam::Vector(5) << 10, 10, 10, 10, 10).finished();
    auto noisemodel = gtsam::noiseModel::Diagonal::Sigmas(v5p);
    gtsam::EssentialMatrixConstraint emc(s0, s1, E, noisemodel);
    
    gtsam::Vector err = emc.evaluateError(gtsam::Pose3::identity(), aPb);
    std::cout << "E error: " << err.transpose() << std::endl;
}

void
TestE::
testDistanceToEpipolarLine()
{
    gtsam::Pose3 pose_t2(gtsam::Rot3::Expmap(gtsam::Vector3(-1.251, -1.1194, 1.073)), gtsam::Point3(7.3938, -35.725, -0.061478));
    gtsam::Pose3 pose_t1(gtsam::Rot3::Expmap(gtsam::Vector3(-1.2479, -1.1258, 1.0751)), gtsam::Point3(7.2857, -36.169, -0.063827));
    Camera axisptz = ParseBoatSurvey::GetCamera();
    gtsam::Cal3_S2::shared_ptr cam = axisptz.GetGTSAMCam();
    
    gtsam::Pose3 btwn = pose_t2.between(pose_t1);
    gtsam::Unit3 aTb(btwn.translation());
    //Create initial estimate
    gtsam::EssentialMatrix trueE(btwn.rotation(), aTb);
    
    for(int i=0; i<p2d0.size(); ++i)
    {
        gtsam::Point2 c0 = p2d0[i]; //cam->calibrate(p2d0[i]);//
        gtsam::Point2 c1 = p2d1[i]; //cam->calibrate(p2d1[i]);//
        
        gtsam::Vector3 p0h = gtsam::EssentialMatrix::Homogeneous(c0);
        gtsam::Vector3 p1h = gtsam::EssentialMatrix::Homogeneous(c1);
        
        gtsam::Vector3 v01 = trueE.matrix() * p0h;
        gtsam::Vector3 v10 = trueE.matrix().transpose() * p1h;
        
        double dist01 = fabs(v01.dot(p1h)) / sqrt(v01.x() * v01.x() + v01.y() * v01.y());
        double dist10 = fabs(v10.dot(p0h)) / sqrt(v10.x() * v10.x() + v10.y() * v10.y());
        double dx = p0h.transpose() * trueE.matrix().transpose() * p1h;
        
        std::cout << "--------------" << std::endl;
        std::cout << "original p0h: " << p0h.transpose() << std::endl;
        std::cout << "original p1h: " << p1h.transpose() << std::endl;
        std::cout << "         v01: " << v01.transpose() << std::endl;
        std::cout << "         v10: " << v10.transpose() << std::endl;
        std::cout << "         d01: " << dist01 << std::endl;
        std::cout << "         d10: " << dist10 << std::endl;
        std::cout << "         dx: " << dx << std::endl;
        
        
    }
}

void
TestE::
testEssentialMatrixVO()
{
    ParseOptimizationResults POR("/Volumes/Untitled/data/maps/", "140106");
    
    for(int i=110; i<500; ++i)
    {
        std::string base = "/Users/shane/Documents/projects/VO/test_E_correspondences/" + std::to_string(cur_pose_idx) + ".csv";
        if(not FileParsing::Exists(base))
        {
            continue;
        }
        std::vector<std::vector<std::string> > data = ReadCSVFile(base);
        std::vector<gtsam::Point2> p0, p1;
        for(int j=0; j<data.size(); ++j)
        {
            p0.push_back(gtsam::Point2(stod(data[j][0]), stod(data[j][1])));
            p1.push_back(gtsam::Point2(stod(data[j][2]), stod(data[j][3])));
        }
        
        
        
        
        exit(1);
        
    }
    
    
    
}






