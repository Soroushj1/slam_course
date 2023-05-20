function normalize_all_bearings(z)
#  Go over the observations vector and normalize the bearings
#  The expected format of z is [range; bearing; range; bearing; ...]

for i in 2:2:length(z)
   z[i] = normalize_angle(z[i])
end
return z
end