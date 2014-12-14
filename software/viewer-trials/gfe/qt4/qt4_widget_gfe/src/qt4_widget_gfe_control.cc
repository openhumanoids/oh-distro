#include <QtGui/QGridLayout>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QGroupBox>

#include <kinematics/kinematics_model_gfe.h>

#include "qt4/qt4_widget_gfe_control.h"

using namespace std;
using namespace urdf;
using namespace state;
using namespace kinematics;
using namespace qt4;

Qt4_Widget_GFE_Control::
Qt4_Widget_GFE_Control( QWidget * parent ) : QTabWidget( parent ),
                                              _state_gfe(),
                                              _sliders(),
                                              _values(){
  QGridLayout * left_hand_layout = new QGridLayout();
  QGridLayout * right_hand_layout = new QGridLayout();
  QGridLayout * left_arm_layout = new QGridLayout();
  QGridLayout * right_arm_layout = new QGridLayout();
  QGridLayout * left_leg_layout = new QGridLayout();
  QGridLayout * right_leg_layout = new QGridLayout();
  QGridLayout * other_layout = new QGridLayout();

  Kinematics_Model_GFE kinematics_model_gfe;

  unsigned int left_hand_index = 0;
  unsigned int right_hand_index = 0;
  unsigned int other_index = 0;

  map< string, double > joints = _state_gfe.joint_angles();
  for( map< string, double >::const_iterator it = joints.begin(); it != joints.end(); it++ ){
//    const State_GFE_Joint& state_gfe_joint = it->second;
    QString joint_name = QString::fromStdString( it->first );
    QLabel * joint_label = new QLabel( QString::fromStdString( it->first ) );
    QSlider * joint_slider = new QSlider( Qt::Horizontal, this );
    joint_slider->setFixedSize( 350, 20 );
    QLabel * joint_value = new QLabel( QString::number( it->second, 'f', 3 ), this );
    if( joint_name.contains( "left_f" ) ){
      left_hand_layout->addWidget( joint_label, left_hand_index, 0 );
      left_hand_layout->addWidget( joint_slider, left_hand_index, 1 );
      left_hand_layout->addWidget( joint_value, left_hand_index, 2 );
      left_hand_index++;
    } else if ( joint_name.contains( "right_f" ) ){
      right_hand_layout->addWidget( joint_label, right_hand_index, 0 );
      right_hand_layout->addWidget( joint_slider, right_hand_index, 1 );
      right_hand_layout->addWidget( joint_value, right_hand_index, 2 );
      right_hand_index++;
    } else if ( joint_name.contains( "l_arm" ) ){
      if( joint_name.contains( "usy" ) ){
        left_arm_layout->addWidget( joint_label, 0, 0 );
        left_arm_layout->addWidget( joint_slider, 0, 1 );
        left_arm_layout->addWidget( joint_value, 0, 2 );
      } else if ( joint_name.contains( "shx" ) ){
        left_arm_layout->addWidget( joint_label, 1, 0 );
        left_arm_layout->addWidget( joint_slider, 1, 1 );
        left_arm_layout->addWidget( joint_value, 1, 2 );
      } else if ( joint_name.contains( "ely" ) ){
        left_arm_layout->addWidget( joint_label, 2, 0 );
        left_arm_layout->addWidget( joint_slider, 2, 1 );
        left_arm_layout->addWidget( joint_value, 2, 2 );
      } else if ( joint_name.contains( "elx" ) ){
        left_arm_layout->addWidget( joint_label, 3, 0 );
        left_arm_layout->addWidget( joint_slider, 3, 1 );
        left_arm_layout->addWidget( joint_value, 3, 2 );
      } else if ( joint_name.contains( "uwy" ) ){
        left_arm_layout->addWidget( joint_label, 4, 0 );
        left_arm_layout->addWidget( joint_slider, 4, 1 );
        left_arm_layout->addWidget( joint_value, 4, 2 );
      } else if ( joint_name.contains( "mwx" ) ){
        left_arm_layout->addWidget( joint_label, 5, 0 );
        left_arm_layout->addWidget( joint_slider, 5, 1 );
        left_arm_layout->addWidget( joint_value, 5, 2 );
      }
    } else if ( joint_name.contains( "r_arm" ) ){
      if( joint_name.contains( "usy" ) ){
        right_arm_layout->addWidget( joint_label, 0, 0 );
        right_arm_layout->addWidget( joint_slider, 0, 1 );
        right_arm_layout->addWidget( joint_value, 0, 2 );
      } else if ( joint_name.contains( "shx" ) ){
        right_arm_layout->addWidget( joint_label, 1, 0 );
        right_arm_layout->addWidget( joint_slider, 1, 1 );
        right_arm_layout->addWidget( joint_value, 1, 2 );
      } else if ( joint_name.contains( "ely" ) ){
        right_arm_layout->addWidget( joint_label, 2, 0 );
        right_arm_layout->addWidget( joint_slider, 2, 1 );
        right_arm_layout->addWidget( joint_value, 2, 2 );
      } else if ( joint_name.contains( "elx" ) ){
        right_arm_layout->addWidget( joint_label, 3, 0 );
        right_arm_layout->addWidget( joint_slider, 3, 1 );
        right_arm_layout->addWidget( joint_value, 3, 2 );
      } else if ( joint_name.contains( "uwy" ) ){
        right_arm_layout->addWidget( joint_label, 4, 0 );
        right_arm_layout->addWidget( joint_slider, 4, 1 );
        right_arm_layout->addWidget( joint_value, 4, 2 );
      } else if ( joint_name.contains( "mwx" ) ){
        right_arm_layout->addWidget( joint_label, 5, 0 );
        right_arm_layout->addWidget( joint_slider, 5, 1 );
        right_arm_layout->addWidget( joint_value, 5, 2 );
      }
    } else if ( joint_name.contains( "l_leg" ) ){
      if( joint_name.contains( "uhz" ) ){
        left_leg_layout->addWidget( joint_label, 0, 0 );
        left_leg_layout->addWidget( joint_slider, 0, 1 );
        left_leg_layout->addWidget( joint_value, 0, 2 );
      } else if ( joint_name.contains( "mhx" ) ){
        left_leg_layout->addWidget( joint_label, 1, 0 );
        left_leg_layout->addWidget( joint_slider, 1, 1 );
        left_leg_layout->addWidget( joint_value, 1, 2 );
      } else if ( joint_name.contains( "lhy" ) ){
        left_leg_layout->addWidget( joint_label, 2, 0 );
        left_leg_layout->addWidget( joint_slider, 2, 1 );
        left_leg_layout->addWidget( joint_value, 2, 2 );
      } else if ( joint_name.contains( "kny" ) ){
        left_leg_layout->addWidget( joint_label, 3, 0 );
        left_leg_layout->addWidget( joint_slider, 3, 1 );
        left_leg_layout->addWidget( joint_value, 3, 2 );
      } else if ( joint_name.contains( "uay" ) ){
        left_leg_layout->addWidget( joint_label, 4, 0 );
        left_leg_layout->addWidget( joint_slider, 4, 1 );
        left_leg_layout->addWidget( joint_value, 4, 2 );
      } else if ( joint_name.contains( "lax" ) ){
        left_leg_layout->addWidget( joint_label, 5, 0 );
        left_leg_layout->addWidget( joint_slider, 5, 1 );
        left_leg_layout->addWidget( joint_value, 5, 2 );
      }
    } else if ( joint_name.contains( "r_leg" ) ){
      if( joint_name.contains( "uhz" ) ){
        right_leg_layout->addWidget( joint_label, 0, 0 );
        right_leg_layout->addWidget( joint_slider, 0, 1 );
        right_leg_layout->addWidget( joint_value, 0, 2 );
      } else if ( joint_name.contains( "mhx" ) ){
        right_leg_layout->addWidget( joint_label, 1, 0 );
        right_leg_layout->addWidget( joint_slider, 1, 1 );
        right_leg_layout->addWidget( joint_value, 1, 2 );
      } else if ( joint_name.contains( "lhy" ) ){
        right_leg_layout->addWidget( joint_label, 2, 0 );
        right_leg_layout->addWidget( joint_slider, 2, 1 );
        right_leg_layout->addWidget( joint_value, 2, 2 );
      } else if ( joint_name.contains( "kny" ) ){
        right_leg_layout->addWidget( joint_label, 3, 0 );
        right_leg_layout->addWidget( joint_slider, 3, 1 );
        right_leg_layout->addWidget( joint_value, 3, 2 );
      } else if ( joint_name.contains( "uay" ) ){
        right_leg_layout->addWidget( joint_label, 4, 0 );
        right_leg_layout->addWidget( joint_slider, 4, 1 );
        right_leg_layout->addWidget( joint_value, 4, 2 );
      } else if ( joint_name.contains( "lax" ) ){
        right_leg_layout->addWidget( joint_label, 5, 0 );
        right_leg_layout->addWidget( joint_slider, 5, 1 );
        right_leg_layout->addWidget( joint_value, 5, 2 );
      }
    } else {
      other_layout->addWidget( joint_label, other_index, 0 );
      other_layout->addWidget( joint_slider, other_index, 1 );
      other_layout->addWidget( joint_value, other_index, 2 );
      other_index++;
    }
    _sliders.insert( make_pair( joint_name.toStdString(), joint_slider ) );
    _values.insert( make_pair( joint_name.toStdString(), joint_value ) );

    if ( kinematics_model_gfe.model().getJoint( joint_name.toStdString() ) ){
      if ( kinematics_model_gfe.model().getJoint( joint_name.toStdString() )->limits ){
        Vector3 axis = kinematics_model_gfe.model().getJoint( joint_name.toStdString() )->axis;
        joint_slider->setRange( kinematics_model_gfe.model().getJoint( joint_name.toStdString() )->limits->lower * 100,
                                kinematics_model_gfe.model().getJoint( joint_name.toStdString() )->limits->upper * 100 );
      }
    }
    connect( joint_slider, SIGNAL( valueChanged( int ) ), this, SLOT( _update_joints( int ) ) ); 
  }

  QGroupBox * left_hand_group_box = new QGroupBox( "left hand" );
  left_hand_group_box->setLayout( left_hand_layout );
  QGroupBox * right_hand_group_box = new QGroupBox( "right hand" );
  right_hand_group_box->setLayout( right_hand_layout);
  QGroupBox * left_arm_group_box = new QGroupBox( "left arm" );
  left_arm_group_box->setLayout( left_arm_layout );
  QGroupBox * right_arm_group_box = new QGroupBox( "right arm" );
  right_arm_group_box->setLayout( right_arm_layout);
  QGroupBox * left_leg_group_box = new QGroupBox( "left leg" );
  left_leg_group_box->setLayout( left_leg_layout );
  QGroupBox * right_leg_group_box = new QGroupBox( "right leg" );
  right_leg_group_box->setLayout( right_leg_layout );
  QGroupBox * other_group_box = new QGroupBox( "other" );
  other_group_box->setLayout( other_layout );

  addTab( left_leg_group_box, "left leg" );
  addTab( left_hand_group_box, "left hand" );
  addTab( left_arm_group_box, "left arm" );
  addTab( other_group_box, "torso" );
  addTab( right_arm_group_box, "right arm" );
  addTab( right_hand_group_box, "right hand" );
  addTab( right_leg_group_box, "right leg" );
}

Qt4_Widget_GFE_Control::
~Qt4_Widget_GFE_Control() {

}

Qt4_Widget_GFE_Control::
Qt4_Widget_GFE_Control( const Qt4_Widget_GFE_Control& other ) {

}

Qt4_Widget_GFE_Control&
Qt4_Widget_GFE_Control::
operator=( const Qt4_Widget_GFE_Control& other ) {

  return (*this);
}

void
Qt4_Widget_GFE_Control::
update_state( State_GFE& stateGFE ){
  for( map< string, QSlider* >::const_iterator it1 = _sliders.begin(); it1 != _sliders.end(); it1++ ){
    map< string, QLabel* >::const_iterator it2 = _values.find( it1->first );
    it2->second->setText( QString::number( _state_gfe.joint( it1->first ).position() * 100.0, 'f', 3 ) );
    it1->second->setValue( _state_gfe.joint( it1->first ).position() * 100.0 );
  }
  update();
  return;
}

void
Qt4_Widget_GFE_Control::
_update_joints( int value ){
  for( map< string, QSlider* >::const_iterator it1 = _sliders.begin(); it1 != _sliders.end(); it1++ ){
    map< string, QLabel* >::const_iterator it2 = _values.find( it1->first );
    it2->second->setText( QString::number( it1->second->value() / 100.0, 'f', 3 ) );
    _state_gfe.joint( it1->first ).set_position( it1->second->value() / 100.0 );
  }
  emit state_update( _state_gfe );
  return;
}

namespace qt4 {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_GFE_Control& other ) {
    return out;
  }

}
