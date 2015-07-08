# Code Base for ICRA 2016 Autonomous Suturing Project


## Installation Guide:

1. Clone the suturing code base into a directory:
`git clone https://siddarthsen@bitbucket.org/siddarthsen/icra2016_suturing.git`
2. Add the folder with subfolders to the matlab path
3. External libraries are used to handle meshes. Clone the gptoolbox and add with
subfolders to the matlab path:
`git clone https://github.com/alecjacobson/gptoolbox.git`
4. The signed distance function uses a C++ method and require compilation. Use
homebrew to install Eigen. Note additional dependencies like cmake, boost, and cgal should
be automatically installed if necessary.
`brew install eigen`
5. libigl is a header only library that is necessary to during compilation. It
needs to be cloned into `usr/local/igl`.
`cd /usr/local `
`mkdir igl`
`cd igl`
`git clone https://github.com/libigl/libigl.git`

6. At the current moment only signed_distance.cpp can be compiled. Please modify
the gptoolbox Mex file so that only signed_distance.cpp is compiled. If namespace
errors occur during compile, modify signed_distance.cpp as necessary (usually add
`igl::matlab::` to function calls until the errors go away.)
