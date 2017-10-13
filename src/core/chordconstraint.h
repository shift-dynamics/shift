/**
@file
@author Kristopher Wehage
@date 2016
@remark
THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.
*/
#ifndef _STAR2_CHORDCONSTRAINT_H_
#define _STAR2_CHORDCONSTRAINT_H_

typedef struct s2_chordconstraint_t s2_chordconstraint_t;
typedef struct s2_body_t s2_body_t;
typedef struct s2_frame_t s2_frame_t;
typedef struct s2_joint_t s2_joint_t;


/*!
@brief chordConstraintNode properties

If chord loops are detected, a virtual massless body is inserted and a
chordConstraintNode data structure is created for each chord loop
*/
typedef struct s2_chordconstraint_t {
  int level;
  int index;
  int substructure;

  char* name;
  char* parentBody; //!< name of parentBody (a virtual body)

  char* originalParentBody; //!< name of original bodyFrom

  char* parentFrame; //!< name of frameFrom

  char* originalParentFrame; //!< name of original frameFrom

  char* childBody; //!< name of child body
  char* childFrame; //!< name of child frame

  s2_body_t* parentBodyPointer; /*!< pointer to virtual body */
  s2_body_t* originalParentBodyPointer; /*!< pointer to original bodyFrom
                                             (on the nonvirtual body) */
  s2_frame_t* parentFramePointer; /*!< pointer to parentFrame
                                       (on the virtual body) */
  s2_frame_t* originalParentFramePointer; /*!< pointer to original
                                               frameFrom (on the
                                               nonvirtual body) */

  s2_body_t* childBodyPointer; /*!< pointer to body data structure */
  s2_frame_t* childFramePointer; /*!< pointer to frame data structure */

  s2_joint_t* parentJointPointer;
  s2_joint_t* childJointPointer;

  s2_chordconstraint_t* next;
} s2_chordconstraint_t;


s2_chordconstraint_t* s2_chordconstraint();

#endif
