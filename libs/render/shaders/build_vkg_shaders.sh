# only need in special cases. Cmake is already compiling this files if the option compile shaders is ON and copies the spv to the bin folder during installation. 
# So, most user dont need to run this script.
glslangValidator -V vrglasses4robots_shader.vert -o vrglasses4robots_shader.vert.spv 
glslangValidator -V vrglasses4robots_shader.frag -o vrglasses4robots_shader.frag.spv 

