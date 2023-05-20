function chi2invtable(p, v)
    dist = Chisq(v)
    return quantile(dist, p)
end