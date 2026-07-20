# only need in special cases. Cmake is already compiling this files if the option compile shaders is ON and copies the spv to the bin folder during installation. 
# So, most user dont need to run this script.
glslangValidator -V vkg_shader.vert -o vkg_shader.vert.spv 
glslangValidator -V vkg_shader.frag -o vkg_shader.frag.spv 

