.. title:: Factoring the Constrained Generalized Mass Matrix

.. _generalized_coordinate_partitioning:

Factoring the Constrained Generalized Mass Matrix
=================================================


In this note we review procedures for recursively sparse factoring a generalized mass matrix with appended preconditioned constraint equations and solving for the unknowns. Preconditioning the constraint equations takes place in three steps described below.

But first, what do we mean by preconditioning constraint equations? The
Harwell Subroutine Library (HSL) http://www.hsl.rl.ac.uk/ provides
specialized functions to sparse factor and solve linear systems of
equations of many forms. The HSL MA67 package
http://www.hsl.rl.ac.uk/catalogue/ma67.html sparse factors a symmetric
matrix with zeros on the diagonal into the form , where is block
diagonal with blocks of size 2 and 1, and where one or both entries on
the diagonal of the 2 by 2 matrices may be 0. The logic to put the
equations into this form represents a considerable amount of extra
overhead, and the constraint equations associated with these pivot rows
must be scaled in association with inverting the pivotal matrices.

It isn’t clear from the documentation in the above link whether MA67
will handle a singular matrix, but if it doesn’t, then HSL has another
similar version that will. When the constraint equations do not have
full row rank, the MA67 or alternate package must expend considerably
more effort and expense splitting the equations into primary and
secondary sets. Of course, the bulk factorization and solver parts of
the package are quite efficient, but the whole process can still be
quite complex.

Here we take a different approach by preconditioning the constraint
equations before appending them to the generalized equations of motion.
In this case, each 2 by 2 pivot submatrix will contain two 1s off
diagonal and one or two 0s on diagonal. Because the determinant of these
2 by 2 pivot matrices is always , no additional scaling or modification
of the constraint equations is ever required. Also, since the
preconditioned constraint equations have full row rank, the coefficient
matrix to be factored will have full rank for all well-posed problems.
Preconditioning the constraint equations up front greatly simplifies the
factorization and solver algorithms and reduces the logical overhead to
nearly zero. In essence, constraint preconditioning has split the
constraint factorization away from the constrained equations of motion
to simplify these algorithms. Since external constraint preconditioning
is required for other purposes as well, no efficiency is sacrificed in
performing it separately.

The first preconditioning step divides the model into kinematic
substructures, where the constrained joint variables within a
substructure are not in any other substructure. Thus each substructure
will have its own set of constraint equations, joint variables, and
Jacobian matrix. When the generalized equations of motion and
preconditioned substructures are properly arranged, the block of
equations and variables associated with the current substructure may be
processed as if all remaining substructures didn’t exist. With this
observation, in the uncoupling and solution process, we may temporarily
ignore the kinematic constraint equations for all but the current
substructure being processed. We will illustrate this shortly.

Thus the equations for just one kinematic substructure are required for
developing and illustrating the optimal uncoupling and solution methods.
In addition, when uncoupling the equations and solving for one dependent
variable and its associated Lagrange multiplier within the remaining
substructure, these processes do not depend on or modify any of the
remaining preconditioned constraint equations. Thus we may also
temporarily ignore the remaining portion of the substructure’s
preconditioned constraint equations, which further simplifies developing
and describing the factorization and solution processes.

The second step of preconditioning the constraint equations applies
Gaussian Elimination with Complete Pivoting (GECP) to the Jacobian
matrix of each kinematic substructure to obtain a Row-Reduced Echelon
Form (RREF) of the constraint equations and Jacobian matrices. This
procedure converts the substructure’s constraint equations into the
uncoupled form

which is also expressed in the factored block matrix form

The symbol represents a column matrix of dependent variables and a
column matrix of independent variables. The symbol represents a column
matrix of constraint accelerations, which are quadratic in first time
derivatives of the joint variables. The symbol represents a matrix of
coefficients relating the dependent variables to the independent
variables, and the symbol represents an identity matrix. Quantities
inside square brackets represent collections of block submatrices. We
will review later how sparse matrix methods are used to compute the and
matrices from the substructure’s primary constraint equations and
Jacobian matrix.

The preconditioned constraint equation for the dependent variable may be
split off from the remainder of Eq. as

where is the row of . This equation is expressed in factored block
matrix form as

Here and are scalars and is a row matrix.

The GECP process has effectively accomplished Generalized Coordinate
Partitioning (GCP) within each kinematic substructure by permuting and
splitting its joint variables into dependent and independent sets.
Rearranging the variables associated with each substructure into
dependent and independent submatrices and likewise within the composite
array of variables requires permuting the rows and columns of the
generalized mass matrix as well. This is addressed in the third
preconditioning step.

The third preconditioning step rearranges the variables, constraint
equations, and generalized equations of motion, placing all independent
variables before the dependent variables within each kinematic
substructure. We first note that optimal equation and variable ordering
within a set of unconstrained generalized equations of motion with
positive definite mass matrix yields zero fills in the factorization
process and minimizes the operations count. However, the constrained
generalized mass matrix is allowed to be singular for well-posed
problems. Because the constrained generalized mass matrix is generally
singular, robust and stable algorithms require a modified solution order
that produces fills and reduces efficiency. The challenge is to find a
good tradeoff between robustness, stability, and efficiency, which is
the primary goal of this discussion.

Before GECP is applied for the first time to the Jacobian matrices to
precondition the constraint equations and obtain GCP, the equations and
variables in the generalized equations of motion are optimally ordered
to minimize fills. However, as indicated in the previous paragraph, the
constrained equations of motion are rarely uncoupled in this order
because the generalized mass matrix may be singular, and attempting to
invert too small pivots or zero pivots must be avoided. Many
researchers, striving to uncouple the constrained equations of motion in
optimal unconstrained order, may employ tricks to obtain positive
definite generalized mass matrices. Factoring such modified matrices in
this order may result in poorly conditioned solutions and errors.
Furthermore, these errors may be difficult or impossible to detect, and
they may destroy robustness, stability, and efficiency.

To avoid such problems and eliminate the possibility of encountering
zero pivots for well-posed models, all dependent variables within the
kinematic substructure being processed must be eliminated before
starting on the first independent variable. To minimize the number of
fills in the uncoupling process, all dependent variables within each
substructure must remain in their original unconstrained optimal order,
and only the independent variables may be moved ahead of the
corresponding dependent variables. The final order of independent
variables within each substructure doesn’t matter, because the number of
fills will be the same for any order. Keeping the independent variables
in natural order is one option, which is used here. This simply means
that the independent variables within a substructure are shifted forward
but not interchanged in the process. To maintain symmetry within the
generalized mass matrix, the equations of motion must be permuted into
the same order as the rearranged variables.

When the constrained generalized equations of motion are initially set
up, their equations and variables are arranged into the same order as
the bodies and joints in the spanning trees. This means that the
equations and variables for bodies and joints with smaller indices come
before the others. The optimal uncoupling and solution order for this
arrangement is then backward elimination and forward substitution, with
indexing order following the spanning trees. Thus the matrix is factored
contiguously from bottom right to top left, which is opposite the method
normally described in the literature.

In the composite set of constrained equations of motion, if one or more
independent variables and unconstrained equations of motion come after
the last kinematic substructure, these variables may be eliminated and
solved for, independent of all constraint equations. This means that all
preconditioned constraint equations may be ignored during the recursive
factoring and solve steps for these and all subsequent independent
variables that come at later stages. The following set of permuted and
block partitioned equations

shows that nothing projects onto the constraint equations and Jacobian
matrix during the factorization and solve process, and it shows that is
not modified when is eliminated and solved for. Thus all constraint
equations may be ignored in Eq. , and it may be replaced temporarily by

It follows from Eq. that the recursive uncoupling and solution
procedures for these independent equations and variables would be the
same as those applied to unconstrained equations of motion with positive
definite matrix. The only requirement is that the magnitude of each
pivotal element must be checked before inverting it, which is good
practice in all situations. Also, the parent-child list used to
recursively uncouple and solve the unconstrained equations of motion
generally won’t work here. This is because some previous uncoupling
steps may have created fills, and indexing through the parent-child list
won’t pick up the added terms.

Equation shows that the rows and columns of the matrix must be aligned
with the rows and columns of the other submatrices. However, we noted
earlier that the entries in and were reordered one way to maximize
robustness and efficiency in the factorization and solve algorithms, and
the rows and columns of the matrix were reordered another way by GECP.
Thus the rows and columns of the matrix don’t align with the other
submatrices in Eq. .

To fix this problem, the third preconditioning step effectively permutes
the rows and columns of into alignment with the respective rows of and
and other submatrices in Eq. . Likewise this step effectively permutes
the column matrix of preconditioned constraint equations in and the
column matrix of Lagrange multipliers in into alignment with . We use
effectively permutes here because, as we’ll see later, permutation
arrays will be used extensively to avoid unnecessary copying of data
between memory locations in all operations.

For simplicity in the following discussion, let us assume that all row
and column permutations have been performed on the data in accordance
with this third preconditioning step and that the same symbols are used
to represent the rearranged quantities. Let us also assume that the
bottom-most kinematic substructure is ready to be recursively eliminated
from the constrained generalized equations of motion, and also assume
that another substructure exists some distance above it in the
spanning-tree order. The preconditioned and permuted equations for this
partial model are

Block eliminating and from Eq. shows that nothing projects onto the
other constraint equations, and is not involved in these steps. Thus the
constraint equations for this and all other upstream substructures as
well may be momentarily ignored while processing this substructure. The
remaining constrained equations of motion without all other substructure
constraints appended are expressed more compactly in block-partitioned
form as

For the first elimination step in Eq. , the last dependent variable and
its associated Lagrange multiplier have been partitioned from the
remaining constraints and Lagrange multipliers. Inspecting the coupling
matrix in Eq. reveals that this reduction step will not involve the
remaining preconditioned Jacobian matrix, constraint equations, or
Lagrange multipliers. So the fourth block row and column in Eq. may also
be momentarily ignored to obtain the desired block partitioned equations

where , , , , and are scalar quantities. Superscript represents the
current dependent variable and Lagrange multiplier in the kinematic
substructure that are being eliminated and solved for, and superscript
represents the remaining dependent variables in the substructure. If in
Eq. happens to represent the last remaining dependent variable and
Lagrange multiplier being eliminated and solved for in the substructure,
then the third block row and column in Eq. would not exist. Likewise
superscript represents the independent variables in the current
substructure being processed. The recursive algorithm will not start
eliminating and solving for the substructure’s independent variables
until the last dependent variable has been eliminated and solved for.
Once all dependent variables and associated Lagrange multipliers have
been eliminated and solved for, the last three block rows and columns in
Eq. will be gone, and Eq. will take the form

and be ready to eliminate and solve for the substructure’s independent
variables.

Returning to Eq. , the block dimension of the matrix equations was
increased to display this kinematic substructure’s other submatrices,
because there will also be some mass projection onto them. Nothing has
been removed from the original set of constrained generalized equations
of motion; only all remaining preconditioned constraint equations and
Lagrange multipliers have been temporarily ignored because they are not
involved in the current uncoupling and solution step applied to Eq. .
The remaining mass components in the current substructure have been
absorbed into the adjacent submatrices and as if everything were
independent. Even though the remaining variables in are dependent,
projections onto their associated and mass matrices will still take
place as though they were independent. In essence, during this
uncoupling and elimination step, no other preconditioned constraint,
other than the one appearing in Eq. , will project mass onto and , or
onto any other mass submatrix. Nor will any other preconditioned
constraint affect how the current preconditioned constraint projects its
mass. That is why all preconditioned constraints, other than the current
one, may be ignored in the current uncoupling and solve step.

The nice feature of Eq. is that the structurally nonsingular 2 by 2
matrix on the diagonal, and the last two equations and variables in Eq.
may be symbolically eliminated in the factorization and solution process
to obtain the reduced set of equations

where

and

From Eqs. through we may identify the set of backward elimination and
forward substitution operations required for this reduction step as

and

Because the assignment in Eq. overwrites the row matrix , the assignment
in Eq. must be performed first. The modified matrix is then used in the
remaining assignments. The assignments in Eqs. through correspond to
backward elimination and multiplying the transpose of the matrix factor
in Eq. with the equations. The assignments in Eqs. through correspond to
multiplying the 2 by 2 inverse-pivot matrix with the equations to
initialize the variables. And the assignments in Eqs. through correspond
to forward substitution and multiplying of the matrix factor in Eq. with
the initialized variables.

The assignments in Eqs. through are duals of those in Eqs. through , and
advantage will be taken of this when the source and destination memory
pointers are set up in the Algorithm Optimization Processor (AOP).

Only that part of the generalized mass matrix on and below the diagonal
is stored. Thus the assignments in Eqs. and would be modified in
computer algorithms so that only the terms on and below the diagonal are
evaluated and stored. We note that in Eq. is not the transpose of in Eq.
, because was changed in Eq. . However is symmetric because the final
reduced matrix in Eq. is symmetric. In the earlier development, the
symbol was used to represent the modified quantity, but since is stored
back in the space occupied by , we use the symbol with the understanding
that its contents have been modified.

Before continuing on to the AOP algorithm, let us take a quick look at a
typical uncoupling step for an independent variable. For this
illustration we can use the simple block equation

where , , and are scalar quantities. From the second block equation in
Eq. we obtain

and from the first block equation in Eq. we obtain

From Eq. we identify the matrix factor term as , which will overwrite .
But before overwriting with , must be moved to a temporary location
because it is required in Eq. . Thus we would have the assignments

and

Here again we would evaluate only the terms in that are on and below the
diagonal.

The most time consuming part of the above algorithms is sparse factoring
the constrained generalized mass matrix. The recursive assignments in
Eqs. through and in Eq. , which represent a substantial percentage of
the factorization operations, have identical structures. This means that
one destination and two source pointer arrays are sufficient to handle
all of these assignments. Generating arrays of source and destination
pointers for the rest of the recursive assignment operations will be
relatively straightforward.

We can also take advantage of duality between the set of assignments in
Eqs. to and Eq. , and between the set of assignments in Eqs. to and Eq.
. These two sets of dual assignments require the same array of source
pointers to elements of the factored generalized mass matrix and
preconditioned Jacobian matrix. Progression through this source pointer
array in the second set of assignments is just the reverse of that in
the first set of assignments. This observation may save only a small
amount of memory, but anything that reduces memory usage helps reduce
overhead.

The above equations represent all required assignments for which the AOP
must set up source and destination memory pointer arrays that the RTA
will use. However, we first must demonstrate how to factor the sparse,
generally singular generalized mass matrix , whose lower triangular part
on and below the diagonal is stored in a linear array. Since fills will
be generated in the factorization process, we cannot use the
parent-child list to determine the location of nonzero entries or to
figure out where to assign updated terms. Instead we will use a linear
Boolean array, which will have the same size as the linear array holding
the generalized mass matrix.

The AOP will use a permuted version of the parent-child list, expanded
to primitive joint variable resolution, to initialize the Boolean array
at the same time it generates the destination pointers for assigning all
nonzero entries to the linear array containing the generalized mass
matrix. It will assign a 1 or true to each location in the Boolean array
where an entry would be assigned to the array containing the mass matrix
. Then, as the AOP goes through the symbolic steps of factoring , it
will symbolically process the Boolean array, checking each accessed
entry for 1 or true to determine if an operation should be performed.
And each time an assignment would be made to the array holding , the AOP
will assign a 1 or true to that corresponding location in the Boolean
array.

A second Boolean array initially represents the sparsity pattern of each
kinematic substructure’s Jacobian matrix, and eventually part of it will
represent the sparsity pattern of the matrix. The AOP will use this
Boolean array to locate nonzero entries in , and it will set up
operations using only those entries in corresponding to the location of
1 or true entries found in its Boolean array representation. As
discussed earlier, the rows and columns of the matrices are not in the
order required for appending directly to the generalized equations of
motion, but they are in the order determined by GECP and generated by
the sparse factor and solve algorithm. To access the matrix entries,
which are stored in the linear array holding all Jacobian matrices, the
AOP will need to generate some offset information and another
permutation array.

Before describing the sparse evaluate and factor algorithm to be applied
to the constrained generalized equations of motion, it will be helpful
to review the three levels of model setup and processing, denoted here
as L1 through L3.

In level L1, kinematic substructures are identified and bodies and
joints are arranged by substructure to block diagonalize the Jacobian
matrix. A parent-child list is generated to represent the relationship
between all bodies and joints. In L1 is where AOPs identify initial
algorithms that will evaluate the generalized equations of motion and
the constraint equations, which have been partitioned into kinematic
substructures.

GECP is applied to the Jacobian matrices evaluated in L1 to identify
generalized coordinate partitioning and precondition the kinematic
substructure constraint equations. The GECP process effectively reorders
the constraint equations and joint variables from L1 to define L2. This
requires new equation and variable permutation arrays. The primary
Jacobian matrices are stored and uncoupled in sparse form in L2 order,
and the primary constraint equations are stored in L2 order. These
operations require a permutation array from L1 to L2, which is defined
by the GECP algorithm.

In L3, the equations and variables in the generalized equations of
motion from L1 are symmetrically rearranged to move independent
variables ahead of dependent variables within each kinematic
substructure. This requires a permutation array from L1 to L3. In
addition, the rows and columns of each substructure’s matrix must be
permuted from L2 to align with the rearranged dependent and independent
variables in L3. This requires another permutation array from L2 to L3.
Thus L3 requires two permutation arrays, one from L1 and another from
L2.

In L2 is where most of the RTA action on kinematic substructures takes
place. An RTA, running exclusively in L2, evaluates the primary part of
each kinematic substructure’s Jacobian matrix and constraint equations,
sparse factors the matrices, and evaluates the matrices and other
quantities. Prior to this, an AOP will have used various permutation
arrays and parent-child lists to set up arrays of source and destination
pointers that the RTA will use in L2 to access and store data in
appropriate levels as it carries out these operations. Each computed
matrix is evaluated and stored in L2, because the algorithms running in
this level should not have to know in advance about the permuted orders
in L3. Since the elements of were not permuted directly to L3 as they
were evaluated in L2, the AOP will generate source pointers that will
allow the RTA, running in L3, to directly access the entries generated
and stored in L2. There will be no overhead to the RTA in doing this.

A similar situation holds for evaluating and processing the constrained
generalized equations of motion. Back in L1, an AOP will have used
previous permutation arrays and the L1 parent-child list to develop
arrays of source and destination pointers for evaluating the generalized
mass matrix and equations of motion. In L1 the AOP stops there. In L3
the AOP identifies and uses new permutation arrays to redirect the
destination memory pointers in the permuted generalized mass matrix to
new locations. In essence, the generalized mass matrix in L3 is
evaluated in the same order defined in L1, but its evaluated quantities
are stored directly in permuted order according to the new destination
memory pointer arrays generated for L3. This method eliminates
unnecessary movement of data in memory. The AOP, indexing in L3, will
use the appropriate permutation array to access and modify these
quantities directly in L1 as needed. As will be discussed later,
accessing and changing quantities between levels will be a common way of
avoiding copying results between levels. In all previous and subsequent
discussions, when talking about processing data in one level that exist
in other levels, we imply that appropriate permutations between the
levels are being used.

With all matrices evaluated in L2, the permuted generalized mass matrix
and equations of motion evaluated in L3, all numerical quantities are in
place for the RTA to factor and solve the constrained generalized
equations of motion. But it can’t do that until the AOP has generated
the necessary source and destination memory pointer arrays. In essence,
the AOP must step through the entire process just once, using all the
logic and permutation arrays and Boolean matrices it needs to factor and
solve these generalized equations of motion with appended preconditioned
constraints. However, the AOP won’t numerically factor the matrix and
solve these equations, but it will go through the act by checking
entries in the Boolean matrices and modifying them as needed. For each
and every numerical operation that would be performed in the sparse
factor and solve process, the AOP will identify the source and
destination memory pointers needed to accomplish this and add them to
the appropriate source and destination memory pointer arrays. When
finished, the AOP will have gone through the entire process of sparse
factoring and solving the constrained generalized equations of motion
with preconditioned constraints, and without having computed a single
number. But the source and destination memory pointer arrays and other
necessary information will be ready for the RTA to compute many numbers.

With this background we are about ready to describe how the AOP will
accomplish these tasks. The AOP-generated procedures to evaluate the
generalized mass matrix and equations of motion in L1 and the
AOP-generated procedures to evaluate and process the constraint
equations and matrices in L2 were detailed elsewhere and won’t be
covered here. Thus we won’t describe how each entry in the generalized
mass matrix is evaluated in L1, but we will describe how the AOP will
use the parent-child list to determine where each entry would go in L1,
because it has to use this information to figure out where the entries
will be redirected in L3.

This process is encumbered by the fact that entries of on and below the
diagonal are stored in a linear array. In essence, when the row , column
entry of , located somewhere on and below its diagonal and stored in
some location of linear array Ml[], is permuted to a new row , column
location in , where will it go in the permuted linear array Ml[]
version?

Computing that new location is straightforward, but comes with a
pitfall. We assume that is a symmetric by matrix, where index accesses
its first row and column. Thus the row and column indices each run from
0 to in . Likewise the indexing in linear array Ml[] runs from 0 to
n\*(n+1)/2-1 to access all elements stored on and below the diagonal of
. To place the entry in Ml[], where variables j and i represent the row
and column indices, we need to use Ml[j\*(j+1)/2+i] where j>=i. This
last restriction is mandatory, because Ml[] can contain no entry from to
the right of the diagonal. Once we have determined where will go into
Ml[], we then need to determine where the row jp, column ip entry in
will go. It will go into Ml[jp\*(jp+1)/2+ip] if jp>=ip, otherwise it
will go into Ml[ip\*(ip+1)/2+jp].

The recursive algorithm to evaluate all nonzero entries on and below the
diagonal of proceeds row-by-row from bottom to top, and left from the
diagonal in each row. Let represent the parent-child list for L1, which
was expanded to individual primitive joint level. Its entries are stored
in the linear array PC[]. The AOP, when generating assignment pointers
for evaluating entries in, will ignore all primitive joint variables
associated with chord joints because their massless frames will
contribute nothing to the corresponding rows and columns of . Thus, for
each joint variable with zero mass on the diagonal, will have an entire
row and an entire column of zeros intersecting at that location. The
number of such all-zero rows and columns will be equal to the number of
primitive chord joint variables. Because the function memset() is used
to zero out the entire Ml[] array before assignments, these all-zero
rows and columns in and its permuted version will automatically be
accounted for. To make it easy for the following algorithms to detect
and skip the all-zero rows and columns of , each such entry of , which
corresponds to a chord joint variable, will be negated and tested by the
AOP to verify that no assignments have to be performed. (Alternatively a
Boolean array HM[] representing “has mass” could be used. HM[] will be 1
or true for each variable and equation with nonzero mass and will be 0
or false otherwise. The algorithms in the rest of this paper should be
modified to use the Boolean HM[] array in place of negating and testing
entries in the PC[] array.

The following algorithm determines the location of all nonzero entries
of that appear on and below the diagonal.

for(j=n-1;j>=0;j--) //Visit rows from bottom to top

{

if(HM[j] //At least one nonzero entry when true

{

i=PC[j]; //Pointer to next nonzero entry or n

k=j\*(j+1)/2; //Start of jth row

// Ml[k+j]=?; //Diagonal entry

while(i<n) //Row finished when off left side

{

// Ml[k+i]=?; //Row j, col i entry

i=PC[i]; //Continue to next nonzero entry

}

}

}

This algorithm shows that the parent-child list makes it easy to find
all nonzero entries in the generalized mass matrix and to determine
their locations in Ml[]. Next we must determine the locations in Ml[] of
the permuted entries in . For this we need the variable permutation
array from L3 to L1, which is the inverse of the permutation from L1 to
L3. Intuitively it seems we should use , which would be correct if we
had already evaluated and wanted to rearrange its entries into . Then we
would simply set each .

However, our recursive algorithm is set up to index and evaluate entries
of in one specific order as described above. We want to store those
entries directly into as they are generated for ; we don’t want to store
them in and then copy and permute them into . Thus we can’t jump to some
arbitrary location to evaluate , because the generating algorithm
indexing in L1 doesn’t know how to do that. What we need instead is the
assignment , where we are now recursively indexing and evaluating
entries according to the above unpermuted algorithm and assigning them
directly to as . Let PP31[] represent this permutation array, and the
modified algorithm becomes

for(j=n-1;j>=0;j--) //Visit rows from bottom to top

{

if(HM[j] //At least one nonzero entry when true

{

i=PC[j]; //Pointer to next nonzero entry or n

jp=PP31[j]; //Permuted row number

kp=jp\*(jp+1)/2; //Start of jpth row

// Ml[kp+jp]=?; //Permuted diagonal entry

while(i<n) //Row finished when off left side

{

ip=PP31[i]; //Potential permuted col index

if(ip<jp) //Current entry is in Ml[]

{

// Ml[kp+ip]=?; //Permuted row jp, col ip entry

}

else //Interchange permuted row, col order

{

// Ml[ip\*(ip+1)/2+jp]=?; //Permuted row ip, col jp entry

}

i=PC[i]; //Continue to next nonzero entry

}

}

}

This algorithm represents a slight modification of the previous one. The
generalized mass matrix has been symmetrically permuted so that each
diagonal entry in is assigned to the diagonal entry in . Then each
generated nonzero entry to the left of the diagonal in the row in is
directed to the row, column position in . If , the entry will be to the
left of the diagonal and in Ml[], otherwise it will be to the right of
the diagonal and must be reflected around the diagonal into Ml[]. This
reflection is accomplished by interchanging the row and column indices
jp and ip before making the assignment.

With this algorithm in hand we are ready to generate the revised
destination pointer array to store all nonzero entries of into Ml[],
generated in original order in L1 for . For this modification we will
use the variable nmlpdp to count the number of destination pointers
generated, and the array MlPDP[] to store the destination pointers into
Ml[] for the entries. The AOP also needs to store the initial sparsity
pattern of in the Boolean array MB[] for use later, which is the same
size as Ml[]. With these arrays, the previous algorithm is modified to

nmlpdp=0;

for(j=n-1;j>=0;j--) //Visit rows from bottom to top

{

if(HM[j] //At least one nonzero entry when true

{

i=PC[j]; //Pointer to next nonzero entry or n

jp=PP31[j]; //Permuted row number

kp=jp\*(jp+1)/2; //Start of jpth row

// Ml[kp+jp]=?; //Permuted diagonal entry

MlPDP[nmlpdp++]=&Ml[kp+jp]; //Set destination pointer

MB[kp+jp]=1; //Record the nonzero entry

while(i<n) //Row finished when off left side

{

ip=PP31[i]; //Potential permuted col index

if(ip<jp) //Current entry is in Ml[]

{

// Ml[kp+ip]=?; //Permuted row jp, col ip entry

MlPDP[nmlpdp++]=&Ml[kp+ip]; //Set destination pointer

MB[kp+ip]=1; //Record the nonzero entry

}

else //Interchange permuted row, col order

{

// Ml[ip\*(ip+1)/2+jp]=?; //Permuted row ip, col jp entry

MlPDP[nmlpdp++]=&Ml[ip\*(ip+1)/2+jp]; //Set destination pointer

MB[ip\*(ip+1)/2+jp]=1; //Record the nonzero entry

}

i=PC[i]; //Continue to next nonzero entry

}

}

}

With these revised destination pointers, the RTA algorithm can
efficiently evaluate and store in Ml[]. The AOP’s next task will be to
generate arrays of source and destination memory pointers for the RTA to
recursively factor in Ml[]. We will use the assignments in Eqs. through
, and in Eqs. through as templates to determine where to access entries
in Ml[] and J[], and store modified entries back in Ml[]. For this
purpose, the AOP will also access the Boolean arrays in MB[] and JB[],
and it will modify the entries in MB[] as it identifies assignments to
Ml[] during the factorization process. In these processes, the AOP will
not modify J[] or JB[].

This factorization algorithm will function differently than previous
ones, because fills will create additional entries on and below the
diagonal in , in places where new nonzero pivots and new entries in the
matrix factors are stored. The parent-child list cannot detect these
fills, and it wouldn’t apply to this permuted matrix anyway. Instead the
AOP must cycle through the updated Boolean array MB[] to determine where
each nonzero entry in , stored in Ml[], is located. Likewise it must
cycle through JB[] to determine where each nonzero entry in , stored in
J[], is located. As the symbolic factorization progresses, the AOP will
update entries in MB[] to record each assignment to Ml[] and potential
new fills. At each step of the factorization process, the entries in
MB[] will reflect the exact sparsity pattern of the modified matrix.
When the factorization process is complete, MB[] will define exactly the
sparsity pattern of the and matrices, and the AOP will then use the
sparsity patterns in MB[] and JB[] to generate the source and
destination pointer arrays for solving the constrained equations of
motion.

The AOP must navigate through the MB[], JB[], Ml[], and J[] arrays to
set up arrays of source and destination pointers that the RTA will use
to sparse factor and solve the equations. The MB[] and Ml[] arrays are
fairly easy to navigate, but the JB[] and J[] arrays will require some
additional work. The constraint equations and Jacobian matrices were
permuted and block partitioned into kinematic substructures in L1, where
each substructure has its own set of constraint equations, its own block
of variables, and its own Jacobian matrix.

The single array J[] is set up with fixed offsets to hold all of the
kinematic substructure Jacobian submatrices. We use the correct fixed
offset into J[] to access each element of the current kinematic
substructure’s Jacobian matrix. For each kinematic substructure indexed
in L2, the AOP initially evaluates its Jacobian matrix in L1 and applies
GECP to factor it, and occasionally thereafter the AOP may evaluate the
Jacobian matrix in L1 and apply GECP to factor it.

From the GECP’s row and column permutation results, the AOP sets up
arrays of source and destination pointers, which the RTA will then use
to sparse evaluate, factor, and solve only the primary equations in L2.
The RTA will assign only those entries associated with the primary
constraint equations directly to J[] in permuted order in L2. In
essence, the entries in J[] for each kinematic substructure reflect a
new by matrix, where rank represents the number of rows and equations
and represents the number of columns and variables.

The by submatrix in the left-hand part of each modified Jacobian matrix,
after the sparse factor and solve operations, will contain the matrix
below the diagonal and the matrix on and above the diagonal. These two
matrices will be used in all recursive solve operations.

To the right of this submatrix is the remaining by submatrix. As can be
seen in Eq. , the rows and columns of are aligned, respectively, with
the dependent and independent variables in this kinematic substructure.
(However, the rows and columns of the submatrix are in L2 and the rows
and columns of the generalized equations of motion are in L3.) Since
each substructure has its own contiguous block of joint variables, the
variables and equations in related blocks of the permuted generalized
equations of motion will also be contiguous. The only difference is that
the permutation and data storage order in the generalized mass matrix ,
represented by MB[] and stored in Ml[], (L3) and the permutation and
data storage order in the processed kinematic substructures ,
represented by JB[] and stored in J[], (L2) are not the same.

The AOP’s sparse factorization and solve processes index through the
generalized equations of motion in L3. Thus the row and column indices
generated for this purpose are associated with entries in the
generalized mass matrix, which is represented by MB[] and stored in
Ml[], and not the entries in the preconditioned Jacobian matrix , which
is indexed in L2 and represented by JB[] and stored in J[].

Based on the GCP information obtained from applying GECP to the Jacobian
matrices in L1, the AOP effectively permuted and block-partitioned the
equations and variables associated with the constrained generalized
equations of motion by kinematic substructure in L3, with independent
variables preceding dependent variables. The AOP will know where each
kinematic substructure’s independent and dependent variables are located
in the composite variable list, but it won’t know where to locate the
corresponding entries in , which were obtained in L2 and represented by
JB[] and stored in J[].

The AOP must generate a translation table that maps each dependent
variable in L3 to a starting row position of in J[] and JB[] in L2. In a
similar manner, this same translation table will map each independent
variable to its column offset in . Thus for a given dependent and
independent variable pair identified in L3, the sum of these two indices
will point to the correct location of that entry in J[] and JB[] in L2.

Let the array vtoc[] represent this translation table from variables to
constraints. Suppose that integer i represents the current row index
into the generalized mass matrix and identifies one of the dependent
variables in L3. Then vtoc[i] will point to the start of in JB[] and J[]
in L2, which represents the row of corresponding to dependent variable .
Likewise, suppose that integer j represents the current column index
into the generalized mass matrix and identifies one of the kinematic
substructure’s independent variables in L3, say . Then vtoc[j] will give
the column offset in to in J[] and JB[] in L2.

The previous discussion stated that the vtoc[] array applies to the
dependent and independent variable arrangement in L2 and was indexed in
L3, but the AOP indexes vtoc[] in L2 when setting it up. Thus the AOP
must insert the permutation array from L2 to L3 into vtoc[] to obtain
the correct entries in . Denote this permutation array as PP23[]. When
the AOP needs a particular entry from , which corresponds to its row
index i and column index j in the generalized equations of motion
indexed in L3, it will use vtoc[PP23[i]] and vtoc[PP23[j]] to access its
information in J[] and JB[], indexed in L2.

The AOP is now ready to start the sparse factorization process. The
current variable can be one of three basic types: an independent
variable with nonzero mass on the diagonal, a dependent variable with
nonzero mass on the diagonal, or a dependent variable with zero mass on
the diagonal. If the diagonal mass term associated with a dependent
variable is zero, the entire row and column of the generalized mass
matrix containing that term will be zero and no inertia will project
anywhere in the generalized mass matrix. If the diagonal mass term
associated with a dependent variable is nonzero, a few, or up to many
mass terms will project into the generalized mass matrix. If the
diagonal mass term associated with an independent variable is zero, the
pivotal element cannot be inverted, an error must be reported, and the
simulation must be terminated. Otherwise a few, or up to many
projections into the generalized mass matrix will take place.

The AOP must be able to determine the type of each variable, which may
be stored in an integer array vt[] and indexed in L3. When it comes time
for the AOP to process an independent variable, whether in a kinematic
substructure or not, the related factorization and solution processes
will not involve the preconditioned constraints in any way. Thus the AOP
only needs to know that this is an independent variable. It can set each
independent variable’s type to and each dependent variable’s type to the
index number of the kinematic substructure that contains it. The AOP
also needs to know the starting and ending index of independent
variables in the generalized equations of motion associated with each
kinematic substructure. Let the array indks[][2] contain these indices
in L3. The indks[][2] array is indexed by kinematic substructure number
for all dependent variables using the vt[] array.

Before developing the recursive uncoupling algorithm we need to take a
closer look at the mass projections associated with eliminating a
dependent variable. When a kinematic substructure contains two or more
dependent variables, their recursive elimination starts from its last
dependent variable and proceeds to the first. As can be seen in Eq. ,
mass projection in the generalized mass matrix occurs in a horizontal
and vertical band across the matrix. These two bands intersect at the
mass submatrix associated with the kinematic substructure’s independent
variables. The horizontal band of this projection has the form

and the vertical band has the form

When performing the projections associated with Eq. , we see that each
set of projections consists of the product of a column matrix with a row
submatrix in the row of the generalized mass matrix associated with the
dependent variable being eliminated. Since the columns of extend across
the kinematic substructure’s independent variables, the rows of extend
across the rows in the generalized mass matrix associated with these
independent generalized equations of motion. If a particular row entry
in is zero, nothing projects into that corresponding row of the
generalized mass matrix. In Eq. we see that each set of projections
consists of the product of a column submatrix with a row matrix in the
column of the generalized mass matrix associated with the dependent
variable being eliminated. In this case, since the columns of extend
across the kinematic substructure’s independent variables, they also
extend across the columns in the generalized mass matrix associated with
these independent variables. If a particular column entry in is zero,
nothing projects into that corresponding column of the generalized mass
matrix.

What we need to see from this discussion is that the and matrices act
like filters on the respective projected rows and columns stretching
across the generalized mass matrix. And for each nonzero entry in we can
perform all projections in one row and column of the generalized mass
matrix at the same time. Thus we need only process each nonzero entry in
in succession to perform all projections associated with this dependent
variable.

There is only one other issue we must contend with in these projections.
Equation shows that these row and column projections cross the matrix
diagonal, so some of these projections take place above and to the right
of the diagonal. Since the final projected matrices are always
symmetric, and because only terms on and below the diagonal are stored
in Ml[], each entry appearing above and to the right of the diagonal was
evaluated somewhere below the diagonal. Thus each entry above and to the
right of the diagonal must not be evaluated. The logic has to take this
into account and limit the indexing so that all assignments remain on
and below the diagonal.

Finally, the basic recursive uncoupling algorithm is summarized as

ws=n\*(n+1)/2; //Start of workspace

for(i=n-1;i>=0;i--) //Bottom to top by row, diagonal to left

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]<0) //Independent variable

{

if(MB[is+i]) //Nonzero pivot if true

{

Ml[is+i]=1.0/Ml[is+i]; //Invert pivot

for(j=i-1;j>=0;j--) //Process pivot row

{

if(MB[is+j]) //Nonzero when true

{

Ml[ws+j]=Ml[is+j]; //Copy into workspace

Ml[is+j]\*=Ml[is+i]; //Entry in L matrix

}

}

for(j=i-1;j>=0;j--) //Project by row j

{

if(MB[is+j]) //Potential projection when true

{

js=j\*(j+1)/2; //Start of row j

for(k=i-1;k>=0;k--) //Project by col k

{

if(MB[is+k]) //Project when true

{

Ml[js+k]-=Ml[ws+j]\*Ml[is+k]; //Project

MB[js+k]=1; //Record fill

}

}

}

}

}

else //Error: known zero pivot

{

report error;

terminate;

}

}

else //Dependent variable, row i

{

if(MB[is+i]) //Do only if nonzero mass

{

for(j=indks[vt[i]][1];j>=indks[vt[i]][0];j--) //Columns of B^di

{

iJ=vtoc[PP13[i]]+vtoc[PP13[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

{

js=j\*(j+1)/2; //Start of row j

for(k=i-1;k>=0;k--) //Cols in row i from j to 0

{

if(JM[is+k]) //NZ entry in Ml[] when true

{

ks=k\*(k+1)/2; //Start of row k

if(k>j) //Below the diagonal

{

Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project col below diagonal

MB[ks+j]=1; //Record fill

}

else if(k==j) //On the diagonal

{

Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project col on diagonal

Ml[is+k]-=Ml[is+i]\*J[iJ]; //Set L matrix term

Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project row on diagonal

MB[ks+j]=1; //Record fill

}

else //Left of diagonal

{

Ml[js+k]-=Ml[is+k]\*J[iJ]; //Project row left of diag.

MB[js+k]=1; //Record fill

}

}

}

}

}

}

}

}

This next algorithm generates the source and destination memory pointer
arrays.

nmli[0]=nmli[1]=0; //Running count of L terms

nmri[0]=nmri[1]=0; //Running count of R terms

nmd[0]=nmd[1]=0; //Running count of D terms

ws=n\*(n+1)/2; //Start of workspace

for(i=n-1,p=1;i>=0;i--,p++) //Bottom to top by row, diagonal to left

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]<0) //Independent variable

{

if(MB[is+i]) //Nonzero pivot if true

{

//Ml[is+i]=1.0/Ml[is+i]; //Invert pivot

MlPP[p]=&Ml[is+i];

for(j=i-1;j>=0;j--) //Process pivot row

{

if(MB[is+j]) //Nonzero when true

{

//Ml[ws+j]=Ml[is+j]; //Copy into workspace

MlWSDP[nmli[p] ]=&Ml[ws+j];

MLSP[nmli[p]++]=&Ml[is+j];

//Ml[is+j]\*=Ml[is+i]; //Entry in L matrix

}

}

for(j=i-1;j>=0;j--) //Project by row j

{

if(MB[is+j]) //Potential projection when true

{

js=j\*(j+1)/2; //Start of row j

for(k=i-1;k>=0;k--) //Project by col k

{

if(MB[is+k]) //Project when true

{

//Ml[js+k]-=Ml[ws+j]\*Ml[is+k]; //Project

MB[js+k]=1; //Record fill

MRDP[nmri[p] ]=&Ml[js+k];

MRWSDP[nmri[p] ]=&Ml[ws+j];

MRLDP[nmri[p]++]=&Ml[is+k];

}

}

}

}

}

else //Error: known zero pivot

{

report error;

terminate;

}

}

else //Dependent variable, row i

{

if(MB[is+i]) //Do only if nonzero mass

{

for(j=indks[vt[i]][1];j>=indks[vt[i]][0];j--) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

{

js=j\*(j+1)/2; //Start of row j

for(k=i-1;k>=0;k--) //Cols in row i from j to 0

{

if(JM[is+k]) //NZ entry in Ml[] when true

{

ks=k\*(k+1)/2; //Start of row k

if(k>j) //Below the diagonal

{

//Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project col below diagonal

MB[ks+j]=1; //Record fill

MDDP[nmd[p] ]=&Ml[ks+j];

MDSP[nmd[p] ]=&Ml[is+j];

JDSP[nmd[p]++]=&J[iJ];

}

else if(k==j) //On the diagonal

{

//Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project col on diagonal

MDDP[nmd[p] ]=&Ml[ks+j];

MDSP[nmd[p] ]=&Ml[is+k];

JDSP[nmd[p]++]=&J[iJ];

//Ml[is+k]-=Ml[is+i]\*J[iJ]; //Set L matrix term

MDDP[nmd[p] ]=&Ml[is+k];

MDSP[nmd[p] ]=&Ml[is+i];

JDSP[nmd[p]++]=&J[iJ];

//Ml[ks+j]-=Ml[is+k]\*J[iJ]; //Project row on diagonal

MDDP[nmd[p] ]=&Ml[ks+j];

MDSP[nmd[p] ]=&Ml[is+k];

JDSP[nmd[p]++]=&J[iJ];

MB[ks+j]=1; //Record fill

}

else //Left of diagonal

{

//Ml[js+k]-=Ml[is+k]\*J[iJ]; //Project row left of diag.

MDDP[nmd[p] ]=&Ml[js+k];

MDSP[nmd[p] ]=&Ml[is+k];

JDSP[nmd[p]++]=&J[iJ];

MB[js+k]=1; //Record fill

}

}

}

}

}

}

}

nmli[p+1]=nmli[p];

nmri[p+1]=nmri[p];

nmd[p+1] =nmd[p];

}

In this final algorithm, an RTA will step through the above pointer
arrays and carry out the necessary operations to factor the matrix. This
is not that bad for such a complex matrix with dependent and independent
variables occurring in somewhat of a random order.

for(i=0;i<n;i++)

{

if(ind[i])

{

if(\*MlPP[i] > eps)\*MlPP[i]=1.0/\*MlPP[i];

else {report bad pivot & exit}

for(j=nmli[i];j<nmli[i+1];j++){\*MWSDP[j]=\*MLP[j];\*MLP[j]\*=\*MPP[i];}

for(j=nmri[i];j<nmri[i+1];j++)\*MRDP[j]-=\*MRWSDP[j]\*\*MRLDP[j];

}

else for(j=nmd[i];j<nmd[i+1];j++)\*MDDP[j]-=\*MDSP[j]\*\*JDSP[j];

}

With the matrix factors available, the next step is to carry out the
recursive solution to the constrained equations of motion. To see how
this would be performed, we can first describe the factored equations as

from which we obtain

The recursive solution to Eq. is

and

From earlier discussions we recall that the product is the dual of the
product , so we may expect to save computer memory by reusing some
pointer arrays for both operations.

The operations required to execute Eq. for dependent variables are
described in Eqs. through , and for independent variables they are
described in Eq. . The operations required to execute Eq. for dependent
variables are described in Eqs. through , and for independent variables
they are described in Eq. . Finally, the operations required to execute
Eq. for dependent variables are described in Eqs. through , and for
independent variables they are described in Eq. . These equations show
that this solution process will use all nonzero terms in the modified
generalized mass matrix, making up part of the matrix and the matrix,
and all nonzero terms in all matrices, making up the rest of the matrix.

Developing these recursive algorithms will take some effort, because
quantities involved in the solution come from three different levels,
L1, L2, and L3. To determine the correct permutation array to use when
an algorithm indexes an array in Lj, which is stored in Li order, use
the permutation to access its elements in Lj order. The generalized mass
matrix is evaluated in L1 and stored in L3. Thus it is indexed in L3
when factored, and the recursive solution algorithms are also indexed in
L3. The preconditioned constraint equations in and are evaluated and
stored in L2 and indexed in L3, so they must be permuted from L2 to L3
using the permutation array . The generalized forces appearing in are
evaluated and stored in L3, so they require no further permutation. The
recursive solution to these equations, indexing in L3, computes and
accesses joint accelerations in L3, which are stored in L1. Since these
quantities are indexed and modified in L3, they must be permuted from L1
to L3 using the permutation array . The Lagrange multipliers are
evaluated and indexed in L3, but stored in L2. Thus they must be
permuted from L2 to L3 using the permutation array . However, there are
some issues related to evaluating , as there are with the array , whose
elements are also evaluated and indexed in L3, but stored in L2.

The solution algorithm indexes the Lagrange multipliers in L3 and stores
them in L2, and if the user requests Lagrange multipliers associated
with the constraint equations, the nonzero primary Lagrange multipliers
must be evaluated in L2 using the equation

The user expects the primary Lagrange multipliers to be associated with
the original joint definitions in L1, so Eq. must be set up to permute
from L2 back to L1 using the permutation array . Also the solution
algorithm indexes and evaluates in L3, which was evaluated and stored as

in L2, so must be permuted from L2 to L3 using the permutation array .
Shortly we will discuss a second problem associated with these last two
quantities.

Elements from these various permutation arrays are used as indices into
the appropriate matrices to directly access or set quantities between
the various levels. Inserting permutations into the various assignments
will eliminate unnecessary copying of data from one memory location to
another. In essence, when the recursive solution algorithms are being
set up for the AOP, these permutation indices will appear in many of the
assignment statements. However, they will disappear once the AOP has set
up the arrays of source and destination memory pointers that the RTA
will use to directly access the required memory locations. Using arrays
of source and destination memory pointers, the RTA’s overhead will be no
more than if all data had been processed at the same level. In fact, the
RTA should run even faster, because using memory pointer arrays
eliminates most of the complex logic required to set up and execute the
recursive solution algorithms.

Now we are ready to start the recursive solution process. The backward
elimination proceeds from bottom to top in the equations, and the
forward substitution proceeds from top to bottom. Multiplying through by
the inverses may proceed in either direction, so we will do this along
with the backward elimination steps. As the algorithms step through the
equations in either direction, they will encounter dependent or
independent variables along the way. At each step the algorithms must
determine whether the current variable is dependent or independent and
then select the appropriate assignments to carry out the necessary
tasks. This is done using the previous array vt[], which identifies each
independent variable with a and each dependent variable with the
kinematic substructure number containing it. We also noted that there
were three pivot types. Each pivot associated with an independent
variable is a scalar value greater than zero, and its inverse is stored
on the diagonal of the matrix somewhere in Ml[]. The second and third
types of pivot are associated with a dependent variable as a 2 by 2
matrix, where the corresponding scalar mass entry on the diagonal in the
second type is greater than zero and is equal to zero in the third type.
This 2 by 2 matrix will be invertible for any value of the scalar mass
entry. The uncoupling processes associated with nonzero pivots involve
substantially more operations than with zero pivots. The algorithm’s
logic will detect each pivot type and make appropriate adjustments.

Before getting to the algorithms, we must address the second issue
related to the and arrays, which are sized and indexed according to the
number of dependent variables in the model. Thus the submatrices in and
, which are associated with individual kinematic substructures, do not
have the same indices as the submatrices in and . That is because the
and submatrices, associated with the individual substructures, are parts
of the composite and arrays, while the and submatrices are parts of
their own composite and arrays, which have different sizes than the
composite and arrays and different indexing requirements. The recursive
algorithms compute indices into the composite and arrays to access
elements of the and submatrices as they progress through the
substructures, but these indices don’t point to the corresponding
elements in the and arrays.

To reduce the complexity of locating elements in the separately indexed
and arrays, an integer array vtod[] is used to access elements of and ,
which are evaluated and stored in L2 using indices evaluated in L3. The
extra entries in vtod[], which correspond to independent variables, have
no meaning and are set to .

With this background, we are ready to present the first version of the
recursive solution algorithm. To eliminate some clutter in this initial
illustration, we will assume that all quantities are expressed directly
in L3 and ignore most permutation arrays. In the second installment we
will include all necessary permutation arrays.

This first major loop does backward elimination and multiplication by
inverse pivots.

for(i=n-1;i>=0;i--) //Bottom to top by row, diagonal to left

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]<0) //Independent variable

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

Ql[j]-=Ml[is+j]\*Ql[i];

ddq[i]=Ml[is+i]\*Ql[i]; //Invert pivot

}

else //Dependent variable, row i

{

for(j=indks[vt[i]][1];j>=indks[vt[i]][0];j--) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

Ql[j]-=J[iJ]\*Ql[i];

}

if(MB[is+i]) //Do only if nonzero mass

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

Ql[j]-=Ml[is+j]\*gd[vtod[i]]; //Needs proper offset here.

}

ddq[i]=gd[vtod[i]];

ld[vtod[i]]=-Ql[i];

if(MB[is+i]) //Do only if nonzero mass

{

ld[vtod[i]]+=Ml[is+i]\*gd[vtod[i]];

}

}

}

This major loop does forward substitution.

for(i=0;i<n;i++) //Top to bottom by row, left to diagonal

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]=<0) //Independent variable

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

ddq[i]-=Ml[is+j]\*ddq[j];

}

else //Dependent variable, row i

{

if(MB[is+i]) //Do only if nonzero mass

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

ld[vtod[i]]+=Ml[is+j]\*ddq[j]; //Needs proper offset here.

}

for(j=indks[vt[i]][0];j<=indks[vt[i]][1];j++) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

ddq[i]-=J[iJ]\*ddq[j];

}

}

}

The above algorithm has set the pattern for recursively solving the
constrained equations of motion in L3. In this next algorithm we include
the permutation arrays for directly accessing the various quantities
from L3.

This first major loop does backward elimination and multiplication by
inverse pivots.

for(i=n-1;i>=0;i--) //Bottom to top by row, diagonal to left

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]<0) //Independent variable

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

Ql[j]-=Ml[is+j]\*Ql[i];

ddq[PP13[i]]=Ml[is+i]\*Ql[i]; //Invert pivot

}

else //Dependent variable, row i

{

for(j=indks[vt[i]][1];j>=indks[vt[i]][0];j--) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

Ql[j]-=J[iJ]\*Ql[i];

}

if(MB[is+i]) //Do only if nonzero mass

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

Q[j]-=Ml[is+j]\*gd[vtod[i]]; //Needs proper offset here.

}

ddq[PP13[i]]=gd[vtod[i]];

ld[vtod[i]]=-Ql[i];

if(MB[is+i]) //Do only if nonzero mass

{

ld[vtod[i]]+=Ml[is+i]\*gd[vtod[i]];

}

}

}

This major loop does forward substitution.

for(i=0;i<n;i++) //Top to bottom by row, left to diagonal

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]=<0) //Independent variable

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

ddq[PP13[i]]-=Ml[is+j]\*ddq[PP13[j]];

}

else //Dependent variable, row i

{

if(MB[is+i]) //Do only if nonzero mass

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

ld[vtod[i]]+=Ml[is+j]\*ddq[PP13[j]]; //Needs proper offset here.

}

for(j=indks[vt[0]][1];j<=indks[vt[i]][1];j++) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

ddq[PP13[i]]-=J[iJ]\*ddq[PP13[j]];

}

}

}

The next step modifies the above algorithms to generate source and
destination memory pointer arrays. This will take some additional effort
to arrange the assignments into the best form for the RTA. We see from
the above algorithm that two recursive loops are executed, the first for
backward elimination and multiplication with inverse pivots, and the
second for forward substitution. Duality between operations involving
the L matrix must be taken into account to use the same array of source
pointers into Ml[] for both loops. In addition, the set of assignment
operations performed in each loop will depend on whether the current
step is associated with an independent or dependent variable. We will
use an if statement on vt[], as done in the above statements.

Since the assignment statements for independent variables have different
structures than for dependent variables, different source and
destination pointer arrays, running operations count arrays, and indexes
will be used for the two. This will minimize the overhead is setting up
and executing the RTA statements.

This first major loop does backward elimination and multiplication by
inverse pivots.

iBB=1; //Index into running count of backward B^di ops

nBB[0]=nBB[1]=0; //Running count of backward B^di ops

iBF=1; //Index into running count of forward B^di ops

nBF[0]=nBF[1]=0; //Running count of forward B^di ops

iMB=1; //Index into running count of backward M ops

nMB[0]=nMB[1]=0; //Running count of backward M ops

iMF=1; //Index into running count of forward M ops

nMF[0]=nMF[1]=0; //Running count of forward M ops

for(i=n-1,p=0;i>=0;i--,p++) //Bottom to top by row, diagonal to left

{

is=i\*(i+1)/2; //Start of row i

ddqDP[p]=&ddq[PP13[i]];

MlPSP[p]=&Ml[is+i];

QlSP[p]=&Ql[i];

gdSP[p]=&gd[vtod[i]];

ldDP[p]=&ld[vtod[i]];

if(vt[i]<0) //Independent variable

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

{

//Ql[j]-=Ml[is+j]\*Ql[i];

QlDP[[nMB[iMB] ]=&Ql[j];

MlSP[[nMB[iMB]++]=&Ml[is+j];

}

nMB[iMB+1]=nMB[iMB++];

nBB[iMB+1]=nBB[iMB++];

//ddq[PP13[i]]=Ml[is+i]\*Ql[i]; //Invert pivot

}

else //Dependent variable, row i

{

for(j=indks[vt[i]][1];j>=indks[vt[i]][0];j--) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

{

//Ql[j]-=J[iJ]\*Ql[i];

QlIDP[nBB[iBB] ]=&Ql[j];

JSP[nBB[iBB]++]=&J[iJ];

}

}

nBB[iBB+1]=nBB[iBB++];

if(MB[is+i]) //Do only if nonzero mass

{

for(j=i-1;j>=0;j--) //Process pivot row

if(MB[is+j]) //Nonzero when true

{

//Ql[j]-=Ml[is+j]\*gd[vtod[i]]; //Needs proper offset here.

QlDP[nMB[iMB] ]=&Ql[j];

MlSP[nMB[iMB]++]=&Ml[is+j];

}

}

nMB[iMB+1]=nMB[iMB++];

//ddq[PP15[i]]=gd[vtod[i]];

//ld[vtod[i]]=-Q[i];

if(MB[is+i]) //Do only if nonzero mass

{

//ld[vtod[i]]+=Ml[is+i]\*gd[vtod[i]];

}

}

}

This major loop does forward substitution.

for(i=0;i<n;i++) //Top to bottom by row, left to diagonal

{

is=i\*(i+1)/2; //Start of row i

if(vt[i]=<0) //Independent variable

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

{

//ddq[PP15[i]]-=Ml[is+j]\*ddq[PP13[j]];

ddqSP[nMF[iMF]++]=&ddq[PP13[j]];

}

nMF[iM+1]=nMF[iMF++];

}

else //Dependent variable, row i

{

if(MB[is+i]) //Do only if nonzero mass

{

for(j=0;j<i;j++) //Process pivot row

if(MB[is+j]) //Nonzero when true

{

//ld[vtod[i]]+=Ml[is+j]\*ddq[PP13[j]]; //Needs proper offset here.

ddqSP[nMF[iMF]++]=&ddq[PP13[j]];

}

}

nMF[iMF+1]=nMF[iMF++];

for(j=indks[vt[0]][1];j<=indks[vt[i]][1];j++) //Columns of B^di

{

iJ=vtoc[PP23[i]]+vtoc[PP23[j]]; //Entry in B^di

if(JB[iJ]) //NZ entry in B^di when true

{

ddq[PP13[i]]-=J[iJ]\*ddq[PP13[j]];

ddqISP[nBF[iBF]++]=&ddq[PP13[j]];

}

}

nBF[iBF+1]=nBF[iBF++];

}

}

This algorithm takes full advantage of duality between the backward
elimination and forward substitution to minimize the number of source
and destination pointer arrays and simplify the RTA algorithm. For an
independent variable, backward elimination evaluates , and forward
substitution evaluates . For a dependent variable, backward elimination
evaluates and , and forward substitution evaluates and . Duality between
these assignments is obvious.

As the AOP steps through the backward elimination process for all
variables, it generates running arrays of source and destination
pointers for the various assignments. The symbols and represent sparse
row matrices with different column dimensions, and each subscript by
itself indicates a column matrix whose row dimension is compatible with
the column dimension of or . Backward elimination indexes from to 0, and
forward substitution indexes from 0 to . At the same time, backward
elimination indexes in reverse order through and , and forward
substitution indexes in forward order through and . Thus also backward
elimination indexes in reverse order through and , and forward
substitution indexes in forward order through and .

Throughout these dual processes, the sequences of all assignments in
forward substitution are exactly the reverse of the sequences of
assignments in backward elimination. Thus the RTA algorithms that
perform forward substitution will be the reverse of those that perform
backward elimination. With this we are ready to describe the RTA
algorithm.

for(p=0;p<n;p++)

{

if(ind[p])

{

for(i=nMB[p];i<nMB[p+1];i++)\*QlDP[i]-=\*MlSP[i]\*\*QlSP[p];

\*ddqDP[p]=\*MlPSP[p]\*\*QSP[p];

}

else

{

for(i=nBB[p];i<nBB[p+1];i++)\*QlIDP[i]-=\*JSP[i]\*\*QlSP[p];

for(i=nMB[p];i<nMB[p+1];i++)\*QlDP[i]-=\*MlSP[i]\*\*qdSP[p];

\*ddqDP[p]=\*qdSP[p];

\*ldDP[p]=-\*QlSP[p];

\*ldDP[p]+=\*MlPSP[p]\*qdSP[p];

}

}

for(p=0;p<n;p++)

{

if(ind[p])

{

for(i=nMF[p];i<nMF[p+1];i++)\*ddqDP[p]-=\*MlSP[i]\*\*ddqSP[i];

}

else

{

for(i=nMF[p];i<nMF[p+1];i++)\*ldDP[p]+=\*MlSP[i]\*\*ddqSP[i];

for(i=nBF[p];i<nBF[p+1];i++)\*ddqDP[p]-=\*JSP[i]\*\*ddqISP[i];

}

}

This RTA algorithm to uncouple and solve the constrained equations of
motion using the previous matrix factors is somewhat longer and more
complex than the earlier factorization algorithm, but that is due to
more variations in assignment statements. Yet it is still much simpler
and quite more efficient than an algorithm using complex logic and
integer indexing arrays. In addition, some source and destination memory
pointer arrays are used more than once for savings in memory space.
