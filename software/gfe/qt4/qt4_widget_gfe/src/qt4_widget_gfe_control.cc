#include <QtGui/QGridLayout>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QGroupBox>

#include "qt4/qt4_widget_gfe_control.h"

using namespace std;
using namespace state;
using namespace qt4;

Qt4_Widget_GFE_Control::
Qt4_Widget_GFE_Control( QWidget * parent ) : QWidget( parent ),
                                              _state_gfe(),
                                              _sliders(),
                                              _values(){
  _state_gfe.from_urdf();

  QGridLayout * left_hand_layout = new QGridLayout();
  QGridLayout * right_hand_layout = new QGridLayout();
  QGridLayout * left_arm_layout = new QGridLayout();
  QGridLayout * right_arm_layout = new QGridLayout();
  QGridLayout * left_leg_layout = new QGridLayout();
  QGridLayout * right_leg_layout = new QGridLayout();
  QGridLayout * other_layout = new QGridLayout();

  unsigned int left_hand_index = 0;
  unsigned int right_hand_index = 0;
  unsigned int left_arm_index = 0;
  unsigned int right_arm_index = 0;
  unsigned int left_leg_index = 0;
  unsigned int right_leg_index = 0;
  unsigned int other_index = 0;

  map< string, State_GFE_Joint > joints = _state_gfe.joints();
  for( map< string, State_GFE_Joint >::const_iterator it = joints.begin(); it != joints.end(); it++ ){
    const State_GFE_Joint& state_gfe_joint = it->second;
    QString joint_name = QString::fromStdString( state_gfe_joint.id() );
    QLabel * joint_label = new QLabel( QString::fromStdString( state_gfe_joint.id() ) );
    QSlider * joint_slider = new QSlider( Qt::Horizontal, this );
    joint_slider->setFixedSize( 100, 20 );
    QLabel * joint_value = new QLabel( QString::number( state_gfe_joint.position(), 'f', 3 ), this );
    if( joint_name.contains( "l_hand" ) ){
      left_hand_layout->addWidget( joint_label, left_hand_index, 0 );
      left_hand_layout->addWidget( joint_slider, left_hand_index, 1 );
      left_hand_layout->addWidget( joint_value, left_hand_index, 2 );
      left_hand_index++;
    } else if ( joint_name.contains( "r_hand" ) ){
      right_hand_layout->addWidget( joint_label, right_hand_index, 0 );
      right_hand_layout->addWidget( joint_slider, right_hand_index, 1 );
      right_hand_layout->addWidget( joint_value, right_hand_index, 2 );
      right_hand_index++;
    } else if ( joint_name.contains( "l_arm" ) ){
      left_arm_layout->addWidget( joint_label, left_arm_index, 0 );
      left_arm_layout->addWidget( joint_slider, left_arm_index, 1 );
      left_arm_layout->addWidget( joint_value, left_arm_index, 2 );
      left_arm_index++;
    } else if ( joint_name.contains( "r_arm" ) ){
      right_arm_layout->addWidget( joint_label, right_arm_index, 0 );
      right_arm_layout->addWidget( joint_slider, right_arm_index, 1 );
      right_arm_layout->addWidget( joint_value, right_arm_index, 2 );
      right_arm_index++;
    } else if ( joint_name.contains( "l_leg" ) ){
      left_leg_layout->addWidget( joint_label, left_leg_index, 0 );
      left_leg_layout->addWidget( joint_slider, left_leg_index, 1 );
      left_leg_layout->addWidget( joint_value, left_leg_index, 2 );
      left_leg_index++;
    } else if ( joint_name.contains( "r_leg" ) ){
      right_leg_layout->addWidget( joint_label, right_leg_index, 0 );
      right_leg_layout->addWidget( joint_slider, right_leg_index, 1 );
      right_leg_layout->addWidget( joint_value, right_leg_index, 2 );
      right_leg_index++;
    } else {
      other_layout->addWidget( joint_label, other_index, 0 );
      other_layout->addWidget( joint_slider, other_index, 1 );
      other_layout->addWidget( joint_value, other_index, 2 );
      other_index++;
    }
    _sliders.insert( make_pair( joint_name.toStdString(), joint_slider ) );
    _values.insert( make_pair( joint_name.toStdString(), joint_value ) );
 
    joint_slider->setRange( -314.0, 314.0 );
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

  QGridLayout * main_layout = new QGridLayout();
  main_layout->addWidget( left_hand_group_box, 0, 0 );
  main_layout->addWidget( left_leg_group_box, 0, 1 );
  main_layout->addWidget( left_arm_group_box, 0, 2 );
  main_layout->addWidget( other_group_box, 0, 3 );
  main_layout->addWidget( right_arm_group_box, 0, 4 );
  main_layout->addWidget( right_leg_group_box, 0, 5 );
  main_layout->addWidget( right_hand_group_box, 0, 6 );

  if( parent == 0 ){
    setLayout( main_layout );
  }
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
