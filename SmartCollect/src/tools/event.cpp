/*----------------------------------------------------------------*/
/* class Object   响应事件函数的类必须是从Object派生下来          */
/*----------------------------------------------------------------*/
class Object
{
};

/*----------------------------------------------------------------*/
/* class Event     模板参数为 返回类型 和响应函数参数类型         */
/* 仅实现一个参数的事件响应函数                                   */
/*----------------------------------------------------------------*/

template<typename rtnTtpe,typename ArgType>
class Event
{
    //使每个事件最多关联响应的函数个数
    #define EVENT_LIST_MAX_NUM  (10)
    typedef rtnTtpe (Object::*pMemFunc)(ArgType arg);

public:
    Event()
    {
        m_totalFunc = 0;
        m_obj = NULL;
        for (int i = 0; i < EVENT_LIST_MAX_NUM; i++)
        {
            m_func[i] = NULL;
        }
    }

    //关联回调成员函数
    template <class _func_type>
    void associate(Object *obj, _func_type func)
    {
        m_obj = obj;
        m_func[m_totalFunc] = static_cast<pMemFunc> (func);
        m_totalFunc++;
    }
    //删除事件关联回调成员函数
    template <class _func_type>
    void disAssociate(Object *obj, _func_type func)
    {
        if (obj != m_obj)
        {
            return;
        }

        //查找
        for (int i = 0; i < m_totalFunc; i++)
        {
            if (m_func[i] == func)
            {
                break;
            }
        }

        //移动删除
        for (i ; i < m_totalFunc - 1; i++)
        {
            m_func[i] = m_func[i + 1];
        }

        m_func[i] = NULL;
        m_totalFunc --;
    }

    //执行关联的回调函数
    void sendEvent(ArgType arg)
    {
        for (int i = 0; i < m_totalFunc; i++)
        {
            if (m_func[i] != NULL)
            {
                ((m_obj->*pMemFunc(m_func[i])))(arg);
            }
        }
    }
private:
    Object* m_obj;
    pMemFunc m_func[EVENT_LIST_MAX_NUM];
    int m_totalFunc;
};