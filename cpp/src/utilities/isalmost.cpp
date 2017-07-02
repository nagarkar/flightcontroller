static const bool isalmost(/*typeof matrix*/a, /*typeof matrix*/ b
		/*TYPEOF: tolerance*/ tolerance, /*typeof scalerRet*/ scalerRet) {
	// get length of input matrix a

	int nr = size(a);
	int nc = size(a[0]);
	
	bool test = false;
	
    // check input for consistency
	if (~all(size(a) == size(b))) {
	   if (all(size(b) == [1 1])) {
	      // convert scalar value b to a matrix of size(a)
	      b = b*ones(size(a));
	   }
	   else {
	      // LOG ERROR
	   }
	}

	one = ones(size(b));

	// perform test
	test = (a <= b+tol*one)&(a >= b-tol*one);
	if (scalarRet) {
	    test = all(all(test));
	}
	return test;
}