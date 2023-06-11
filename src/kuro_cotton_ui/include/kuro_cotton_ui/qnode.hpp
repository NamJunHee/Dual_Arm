/**
 * @file /include/kuro_cotton_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef kuro_cotton_ui_QNODE_HPP_
#define kuro_cotton_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
//Msg
#include <msg_generate/kuro_cotton_candy.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_cotton_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    //ros::Publisher cotton_candy_pub;


Q_SIGNALS:

    void rosShutdown();
    void updateUI();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace kuro_cotton_ui

#endif /* kuro_cotton_ui_QNODE_HPP_ */
