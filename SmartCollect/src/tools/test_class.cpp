/*----------------------------------------------------------------*/
/* class TestClass                                                */
/*----------------------------------------------------------------*/
class TestClass:public Object
{
public:
    TestClass()
    {
        //关联事件
        m_event.myEvent.associate(this,&TestClass::executeCb1);
        m_event.myEvent.associate(this,&TestClass::executeCb2);
    }

    //事件响应函数
    bool executeCb1(int result)
    {
        cout<<"executeCb1 result = "<<result<<endl;
        return true;
    }
    //事件响应函数
    bool executeCb2(int result)
    {
        cout<<"executeCb2 result = "<<result<<endl;
        return true;
    }

    void execute()
    {
        m_event.test();
    }
    void stop()
    {
        //删除事件关联函数
        m_event.myEvent.disAssociate(this,&TestClass::executeCb1);
    }
private:
    TestEvent m_event;
};