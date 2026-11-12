#include <gtest/gtest.h>
#include <vikit/cameras/equidistant_distortion.h>
#include <array>
#include <cmath>

TEST(EquidistantEgo, FullImageGridEdgesAndPrincipalPoint)
{
  const std::array<std::array<double, 8>, 2> cameras{{
    {{682.0539920133708, 682.3327235394054, 976.7157606046939, 550.2659094669332,
      0.3966402368796761, 0.18311725738817283, -0.20432628115126497, 0.11014047407355965}},
    {{678.8759785509483, 679.1784954472535, 989.1676113493628, 513.1845551578026,
      0.3983529379855351, 0.1704791309633979, -0.1881098669606969, 0.10218292444234162}}
  }};
  for (const auto& c : cameras)
  {
    vk::cameras::EquidistantDistortion distortion(c[4],c[5],c[6],c[7]);
    auto check = [&](double u, double v)
    {
      double x = (u-c[2])/c[0], y = (v-c[3])/c[1];
      const double rd = std::hypot(x,y);
      distortion.undistort(x,y);
      ASSERT_TRUE(std::isfinite(x) && std::isfinite(y));
      const double z = 1.0/std::sqrt(1.0+x*x+y*y);
      EXPECT_GT(z,0.48);
      // Independent scalar bisection and forward polynomial, no inverse helper.
      double lower=0.0, upper=std::acos(-1.0)/2.0;
      for (int i=0;i<100;++i)
      {
        const double t=(lower+upper)/2.0, t2=t*t;
        const double projected=t*(1.0+t2*(c[4]+t2*(c[5]+t2*(c[6]+t2*c[7]))));
        if(projected<rd) lower=t; else upper=t;
      }
      EXPECT_NEAR(std::atan(std::hypot(x,y)),(lower+upper)/2.0,1e-13);
      distortion.distort(x,y);
      EXPECT_LT(std::hypot(x*c[0]+c[2]-u,y*c[1]+c[3]-v),1e-6);
    };
    for (int row=0;row<37;++row)
      for (int col=0;col<65;++col) check(1919.0*col/64.0,1079.0*row/36.0);
    for(double dx : {-1e-7,0.0,1e-7})
      for(double dy : {-1e-7,0.0,1e-7}) check(c[2]+dx,c[3]+dy);
  }
}
TEST(EquidistantEgo, RejectsNonfiniteInput)
{
  vk::cameras::EquidistantDistortion distortion(0.4,0.18,-0.2,0.11);
  double x=std::numeric_limits<double>::infinity(), y=0;
  EXPECT_THROW(distortion.undistort(x,y),std::domain_error);
}
