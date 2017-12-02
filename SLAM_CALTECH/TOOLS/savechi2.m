global chi2;

alpha = 0.99;
chi2 = chi2inv(alpha,1:1000);

save 'data/chi2' chi2;