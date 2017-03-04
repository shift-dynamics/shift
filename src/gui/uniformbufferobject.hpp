#ifndef _STAR2_UNIFORMBUFFEROBJECT_H_
#define _STAR2_UNIFORMBUFFEROBJECT_H_

#include <qgl.h>

struct UBOInfoStruct {
    QString uniformName;
    GLuint progId;
    GLuint blockIdx;
    GLint blockSize;
    GLuint bindingIdx;
};

typedef UBOInfoStruct UBOInfo;

class UniformBufferObject
{
public:
    UniformBufferObject();

    bool create();
    bool isCreated() const;

    void destroy();

    bool bind();
    bool bind(GLuint progId, QString uniformName);
    void release();

    GLuint bufferId() const;

//    bool read(int offset, void *data, int count);
    void write(int offset, const void *data, int count);

    void allocate(const void *data, int count);
    inline void allocate(int count) { allocate(0, count); }


protected:
    static GLuint bindingIndex();
    static QVector<GLuint> m_bindingIndices;

    bool initFunctionPointers(const QGLContext* m_glContext);

    GLuint m_bufferId;

    const QGLContext* m_glContext;

    bool m_inited;

    QVector<UBOInfo> m_UBOInfos;

    PFNGLBINDBUFFERPROC glBindBuffer;
    PFNGLBINDBUFFERBASEPROC glBindBufferBase;
    PFNGLBINDBUFFERRANGEPROC glBindBufferRange;
    PFNGLBUFFERDATAPROC glBufferData;
    PFNGLBUFFERSUBDATAPROC glBufferSubData;
    PFNGLDELETEBUFFERSPROC glDeleteBuffers;
    PFNGLGENBUFFERSPROC glGenBuffers;
    PFNGLGETACTIVEUNIFORMBLOCKIVPROC glGetActiveUniformBlockiv;
    PFNGLGETACTIVEUNIFORMSIVPROC glGetActiveUniformsiv;
    PFNGLGETUNIFORMBLOCKINDEXPROC glGetUniformBlockIndex;
    PFNGLGETUNIFORMINDICESPROC glGetUniformIndices;
    PFNGLUNIFORMBLOCKBINDINGPROC glUniformBlockBinding;
};

#endif // _UNIFORMBUFFEROBJECT_H_
