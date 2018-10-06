
# Discrete Wind Gust Model
#
# Adapted to python, original model can be found here:
# https://au.mathworks.com/help/aeroblks/discretewindgustmodel.html
def wind_model(gust_amplitude, gust_length, dist_travelled):
    if(dist_travelled < 0):
        wind_res = 0;
    elif(dist_travelled < gust_length):
        wind_res = (gust_amplitude / 2) * (1.0 - cos((pi * dist_travelled) / gust_length));
    else:
        wind_res = gust_amplitude;

    return wind_res;