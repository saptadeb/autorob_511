//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
    // returns 2D array that is the result of m1*m2

    var res = new Array(m1.length)    // creating a placeholder
    // console.log(m1,m2)
    for (i = 0; i < m1.length; i++){
        res[i] = new Array(m2[0].length)
        for (j = 0; j < m2[0].length; j++){
            var temp = 0
            for (k = 0; k < m2.length; k++){
                temp += m1[i][k] * m2[k][j]
            }
            res[i][j] = temp
        }
    }
    return res
}

function matrix_transpose(m) {
    // returns 2D array transpose

    var res = new Array(m[0].length)    // creating a placeholder
    
    for (i = 0; i < m[0].length; i++){
        res[i] = [];
    }

    for (i = 0; i < m.length; i++){
        // res[i] = new Array(m[0].length)
        for (j = 0; j < m[0].length; j++){
            res[j][i] = m[i][j]
        }
    }
    return res

}

function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m
    var res = [];
    var m_trans = matrix_transpose(m);
    
    if (m.length == m[0].length){
        res = numeric.inv(m);
    } else if (m.length > m[0].length){
        // console.log(m.length, m[0].length)
        // console.log(m_trans.length, m_trans[0].length,m.length,m[0].length)
        var temp = matrix_multiply(m_trans,m);
        // console.log(temp)
        // console.log(numeric.inv(temp))
        res = matrix_multiply(numeric.inv(temp), m_trans);
    } else{
        // console.log(m.length, m[0].length)
        var temp = matrix_multiply(m,m_trans);
        res = matrix_multiply(m_trans, numeric.inv(temp));
    }
    return res;
}

function matrix_invert_affine(m) {
    // returns 2D array that is the invert affine of 4-by-4 matrix m

    var rot = matrix_copy(m).splice(0,3)
    var trans = matrix_copy(m).splice(0,3)

    for (i = 0; i < 3; i++){
        rot[i] = rot[i].splice(0,3)
        trans.splice(0,3)
    }

    var rot_T = matrix_transpose(rot)
    var temp = matrix_multiply(rot_T, trans)

    var res = new Array(m.length)

    for (i = 0; i < 3; i++){
        res[i] = ret_T[i].slice()
        res[i].push(temp[i][0])
    }
    res[3] = [0,0,0,1]    
    return res
}

function vector_normalize(v) {
    // returns normalized vector for v

    var v_copy = new Array(v.length)
    var temp = 0
    for (i = 0;i < v.length; i++){
        temp = temp + v[i] * v[i]
    }
    temp = Math.sqrt(temp)

    for (i = 0; i < v.length; i++){
        if (Math.abs(temp) < Number.EPSILON){
            v_copy[i] = 0
        } else{
            v_copy[i] = v[i] / temp
        }
    }
    return v_copy
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions

    var res = new Array(a.length)
    res[0] = a[1] * b[2] - a[2] * b[1]
    res[1] = -(a[0] * b[2] - a[2] * b[0])
    res[2] = a[0] * b[1] - a[1] * b[0]
    return res
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix
    var res = new Array(4)
    for (i = 0; i < 4; i++){
        res[i] = new Array(4)
        for (j = 0; j < 4; j++){
            if (i != j){
                res[i][j] = 0
            }
            else{
                res[i][j] = 1
            }
        }
    }
    return res;

}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    var temp = generate_identity()

    temp[0][3] = tx
    temp[1][3] = ty
    temp[2][3] = tz

    return temp
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians

    var temp = generate_identity();
    temp[1][1] = Math.cos(angle);
    temp[2][2] = Math.cos(angle);
    temp[1][2] = - Math.sin(angle);
    temp[2][1] = Math.sin(angle);
    return temp; 
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    
    var temp = generate_identity()
    temp[0][0] = Math.cos(angle)
    temp[2][2] = Math.cos(angle)
    temp[0][2] = Math.sin(angle)
    temp[2][0] = - Math.sin(angle)
    return temp
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    
    var temp = generate_identity()
    temp[0][0] = Math.cos(angle)
    temp[1][1] = Math.cos(angle)
    temp[0][1] = - Math.sin(angle)
    temp[1][0] = Math.sin(angle)
    return temp
}

function generate_rotation_matrix (r,p,y){
    var m = matrix_multiply (generate_rotation_matrix_Z( y ), generate_rotation_matrix_Y( p ) );
    m = matrix_multiply (m, generate_rotation_matrix_X( r ) );
    return m
}

function generate_transformation(xyz, rpy) {
    return matrix_multiply(generate_translation_matrix(xyz[0], xyz[1], xyz[2]),
        matrix_multiply(generate_rotation_matrix_Z(rpy[2]),
            matrix_multiply(generate_rotation_matrix_Y(rpy[1]),
                generate_rotation_matrix_X(rpy[0]))));
}

function rotation_matrix_to_axisangle(R) {
    var thetaX, thetaY, thetaZ;

    thetaX = Math.atan2(R[2][1], R[2][2]);
    thetaZ = Math.atan2(R[1][0], R[0][0]);
    thetaY = Math.atan2(-R[2][0],Math.pow(Math.pow(R[2][1], 2)+ Math.pow(R[2][2], 2), 0.5));

    return [thetaX, thetaY, thetaZ];
}

function vector_dot(v1,v2){
    var v = new Array(v1.length);
    for (i=0;i<v1.length;++i){
        v[i]=v1[i]*v2[i];
    }
    return v;
}

function matrix_from_vector(v){
    var mat = [];
    for(var i = 0; i < v.length; ++i){
        mat.push([]);
        mat[i].push(v[i]);
    }
    return mat;
}

function matrix_vec_multiply(m1,v1){
    var v = [];
    var i;
    var j;
    v1 = [v1[0],v1[1],v1[2],0];
    for (i=0;i<m1.length;++i){
        v[i] = 0;
        for (j=0;j<m1[0].length;++j){
            v[i] += m1[i][j]*v1[j];
        }
    }
    v = [v[0],v[1],v[2]];
    return v;
}

function vec_minus(v1,v2) {
    var res = [];
    for (var i = 0; i < v1.length; i++) {
        res.push(v1[i] - v2[i]);
    }
    return res;
}

function rev_vec(vec) {
    var res = [];
    var n = vec.length;
    for (var i = n - 1; i>=0; i--) {
        res[n-1-i] = vec[i];
    }
    return res;
}


// **** ADVANCED EXTENSION: LU Decomposition ****//

function matrix_inverse(A) {
    // ---------------------------------------------------------------------------
    // A                    // n x n matrix
    // return A^(-1)        // n x n matrix
    // ---------------------------------------------------------------------------

    var i, j, k;
    if (A.length !== A[0].length) {
        return matrix_pseudoinverse(A);
    }

    var p = lu_decomposition(A);
    var P = p.P;
    var L = p.L;
    var U = p.U;

    var invL = generate_identity(L.length);
    var invU = generate_identity(U.length);
    for (i = 0; i < A.length; i++) {
        for (k = i + 1; k < A.length; k++) {
            for (j = i; j <= k - 1; j++) {
                invL[k][i] -= L[k][j] * invL[j][i];
            }
        }
    }
    for (i = 0; i < A.length; i++) {
        invU[i][i] = 1 / U[i][i];
        for (k = i - 1; k >= 0; k--) {
            var tmp = 0;
            for (j = k + 1; j <= i; j++) {
                tmp += U[k][j] * invU[j][i];
            }
            invU[k][i] = -tmp / U[k][k];
        }
    }
    return matrix_multiply(matrix_multiply(invU, invL), P);
}

function lu_decomposition(A) {
    // A: n x n matrix
    // return {"P": P, "L": L, "U": U}
    var n = A.length;
    var P = generate_identity(n);
    var L = generate_identity(n);
    var U = matrix_copy(A);

    for (var i = 0; i < n - 1; i++) {
        var maxIndex = getMaxIndexInColumn(U, i);

        if (i !== maxIndex) {
            for (var j = i; j < n; j++) {
                var tmp = U[i][j];
                U[i][j] = U[maxIndex][j];
                U[maxIndex][j] = tmp;
            }
            for (j = 0; j < i; j++) {
                tmp = L[i][j];
                L[i][j] = L[maxIndex][j];
                L[maxIndex][j] = tmp;
            }
            tmp = P[i];
            P[i] = P[maxIndex];
            P[maxIndex] = tmp;
        }

        for (j = i + 1; j < n; j++) {
            tmp = U[j][i] / U[i][i];
            L[j][i] = tmp;
            for (var k = i; k < n; k++) {
                U[j][k] -= tmp * U[i][k];
            }
        }
    }

    return {"P": P, "L": L, "U": U};

    function getMaxIndexInColumn(U, j) {
        var max = Math.abs(U[j][j]);
        var maxIndex = j;
        for (var i = j + 1; i < U.length; i++) {
            var entry = Math.abs(U[i][j]);
            if (entry > max) {
                max = entry;
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}

function linear_solve(A, b) {
    // return x, where Ax = b
    var i, j, sum;
    var p = lu_decomposition(A);
    var P = p.P;
    var L = p.L;
    var U = p.U;

    var P_b = matrix_multiply(P, b);

    var U_x = [];
    for (i = 0; i < b.length; i++) {
        U_x[i] = [];
        sum = 0;
        for (j = 0; j < i; j++) {
            sum += L[i][j] * U_x[j][0];
        }
        U_x[i][0] = P_b[i][0] - sum;
    }

    var x = [];
    for (i = b.length - 1; i >= 0; i--) {
        sum = 0;
        for (j = i + 1; j < b.length; j++) {
            sum += U[i][j] * x[j - i - 1][0];
        }
        x.unshift([(U_x[i][0] - sum) / U[i][i]]);
    }
    return x;
}