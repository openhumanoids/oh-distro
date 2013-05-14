#include <QtGui/QGroupBox>
#include <QtGui/QGridLayout>

#include <authoring/qt4_widget_constraint_task_space_region_editor.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace affordance;
using namespace authoring;

Qt4_Widget_Constraint_Task_Space_Region_Editor::
Qt4_Widget_Constraint_Task_Space_Region_Editor( Constraint_Task_Space_Region * constraint, 
                                              Model& robotModel,
                                              vector< AffordanceState >& affordanceCollection, 
                                              QWidget * parent ) : QWidget( parent, Qt::Dialog ),
                                                                    _constraint( constraint ),
                                                                    _robot_model( robotModel ),
                                                                    _robot_affordances(),
                                                                    _object_affordances( affordanceCollection ),
                                                                    _label_id( new QLabel( "N/A", this ) ),
                                                                    _combo_box_parent( new QComboBox( this ) ),
                                                                    _combo_box_child( new QComboBox( this ) ),
                                                                    _combo_box_type( new QComboBox( this ) ),
                                                                    _double_spin_box_x_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_x_max( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_y_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_y_max( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_z_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_z_max( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_roll_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_roll_max( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_pitch_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_pitch_max( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_yaw_min( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_yaw_max( new QDoubleSpinBox( this ) ),
                                                                    _push_button_x_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_x_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _push_button_y_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_y_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _push_button_z_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_z_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _push_button_roll_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_roll_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _push_button_pitch_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_pitch_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _push_button_yaw_min( new QPushButton( QString( "min" ), this ) ),
                                                                    _push_button_yaw_max( new QPushButton( QString( "max" ), this ) ),
                                                                    _double_spin_box_parent_to_constraint_x( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_parent_to_constraint_y( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_parent_to_constraint_z( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_child_to_constraint_x( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_child_to_constraint_y( new QDoubleSpinBox( this ) ),
                                                                    _double_spin_box_child_to_constraint_z( new QDoubleSpinBox( this ) ){

  vector< shared_ptr< Link > > links;
  _robot_model.getLinks( links );
  for( vector< shared_ptr< Link > >::iterator it1 = links.begin(); it1 != links.end(); it1++ ){
    for( map< string, shared_ptr< vector< shared_ptr< Collision > > > >::iterator it2 = (*it1)->collision_groups.begin(); it2 != (*it1)->collision_groups.end(); it2++ ){
      _robot_affordances.push_back( pair< string, string >( (*it1)->name, it2->first ) );
    }
  }

  for( vector< pair< string, string > >::iterator it = _robot_affordances.begin(); it != _robot_affordances.end(); it++ ){
    _combo_box_parent->addItem( QString( "%1-%2" ).arg( QString::fromStdString( it->first ) ).arg( QString::fromStdString( it->second ) ) );
  }

  for( vector< AffordanceState >::const_iterator it = _object_affordances.begin(); it != _object_affordances.end(); it++ ){
    _combo_box_child->addItem( QString::fromStdString( it->getName() ) );
  }

  for ( uint i = 0; i < NUM_CONTACT_TYPES; i++ ) {
      _combo_box_type->addItem( QString::fromStdString( Constraint_Task_Space_Region::contact_type_t_to_std_string( (contact_type_t) i ) ) );
  }

  _double_spin_box_x_min->setSuffix( " m" );
  _double_spin_box_x_max->setSuffix( " m" );
  _double_spin_box_y_min->setSuffix( " m" );
  _double_spin_box_y_max->setSuffix( " m" );
  _double_spin_box_z_min->setSuffix( " m" );
  _double_spin_box_z_max->setSuffix( " m" );
  _double_spin_box_roll_min->setSuffix( " deg" );
  _double_spin_box_roll_max->setSuffix( " deg" );
  _double_spin_box_pitch_min->setSuffix( " deg" );
  _double_spin_box_pitch_max->setSuffix( " deg" );
  _double_spin_box_yaw_min->setSuffix( " deg" );
  _double_spin_box_yaw_max->setSuffix( " deg" );

  _double_spin_box_x_min->setSingleStep( 0.1 );
  _double_spin_box_x_max->setSingleStep( 0.1 );
  _double_spin_box_y_min->setSingleStep( 0.1 );
  _double_spin_box_y_max->setSingleStep( 0.1 );
  _double_spin_box_z_min->setSingleStep( 0.1 );
  _double_spin_box_z_max->setSingleStep( 0.1 );
  _double_spin_box_roll_min->setSingleStep( 11.25 );
  _double_spin_box_roll_max->setSingleStep( 11.25 );
  _double_spin_box_pitch_min->setSingleStep( 11.25 );
  _double_spin_box_pitch_max->setSingleStep( 11.25 );
  _double_spin_box_yaw_min->setSingleStep( 11.25 );
  _double_spin_box_yaw_max->setSingleStep( 11.25 );

  _double_spin_box_x_min->setRange( -1000.0, 1000.0 );
  _double_spin_box_x_max->setRange( -1000.0, 1000.0 );
  _double_spin_box_y_min->setRange( -1000.0, 1000.0 );
  _double_spin_box_y_max->setRange( -1000.0, 1000.0 );
  _double_spin_box_z_min->setRange( -1000.0, 1000.0 );
  _double_spin_box_z_max->setRange( -1000.0, 1000.0 );
  _double_spin_box_roll_min->setRange( -180.0, 180.0 );
  _double_spin_box_roll_max->setRange( -180.0, 180.0 );
  _double_spin_box_pitch_min->setRange( -90.0, 90.0 );
  _double_spin_box_pitch_max->setRange( -90.0, 90.0 );
  _double_spin_box_yaw_min->setRange( -180.0, 180.0 );
  _double_spin_box_yaw_max->setRange( -180.0, 180.0 );

  _double_spin_box_parent_to_constraint_x->setSuffix( " m" );
  _double_spin_box_parent_to_constraint_y->setSuffix( " m" );
  _double_spin_box_parent_to_constraint_z->setSuffix( " m" );

  _double_spin_box_parent_to_constraint_x->setRange( -1000.0, 1000.0 );
  _double_spin_box_parent_to_constraint_y->setRange( -1000.0, 1000.0 );
  _double_spin_box_parent_to_constraint_z->setRange( -1000.0, 1000.0 );

  _double_spin_box_child_to_constraint_x->setSuffix( " m" );
  _double_spin_box_child_to_constraint_y->setSuffix( " m" );
  _double_spin_box_child_to_constraint_z->setSuffix( " m" );

  _double_spin_box_child_to_constraint_x->setRange( -1000.0, 1000.0 );
  _double_spin_box_child_to_constraint_y->setRange( -1000.0, 1000.0 );
  _double_spin_box_child_to_constraint_z->setRange( -1000.0, 1000.0 );

  if( _constraint != NULL ){
    _combo_box_type->setCurrentIndex( _constraint->contact_type() );
    _label_id->setText( QString::fromStdString( _constraint->id() ) );
    if( _constraint->parent().first != "N/A" ){
      for( unsigned int i = 0; i < _robot_affordances.size(); i++ ){
        if( _robot_affordances[ i ].first == _constraint->parent().first ){
          if( _robot_affordances[ i ].second == _constraint->parent().second ){
            _combo_box_parent->setCurrentIndex( i );
          }
        }
      }
    } else {
      if( !_robot_affordances.empty() ){
        _constraint->parent().first = _robot_affordances.begin()->first;
        _constraint->parent().second = _robot_affordances.begin()->second;
      }
    }
    if( _constraint->child() != NULL ){
      for( unsigned int i = 0; i < _object_affordances.size(); i++ ){
        if( _object_affordances[ i ].getName() == _constraint->child()->getName() ){
          _combo_box_child->setCurrentIndex( i );
        }
      }
    } else {
      if( !_object_affordances.empty() ){
        _constraint->child() = &(*_object_affordances.begin());
      }
    }
    _double_spin_box_x_min->setValue( _constraint->ranges()[0].first );
    _double_spin_box_x_max->setValue( _constraint->ranges()[0].second );
    _double_spin_box_y_min->setValue( _constraint->ranges()[1].first );
    _double_spin_box_y_max->setValue( _constraint->ranges()[1].second );
    _double_spin_box_z_min->setValue( _constraint->ranges()[2].first );
    _double_spin_box_z_max->setValue( _constraint->ranges()[2].second );
    _double_spin_box_roll_min->setValue( _constraint->ranges()[3].first );
    _double_spin_box_roll_max->setValue( _constraint->ranges()[3].second );
    _double_spin_box_pitch_min->setValue( _constraint->ranges()[4].first );
    _double_spin_box_pitch_max->setValue( _constraint->ranges()[4].second );
    _double_spin_box_yaw_min->setValue( _constraint->ranges()[5].first );
    _double_spin_box_yaw_max->setValue( _constraint->ranges()[5].second );
    _double_spin_box_parent_to_constraint_x->setValue( _constraint->parent_to_constraint().p[0] );
    _double_spin_box_parent_to_constraint_y->setValue( _constraint->parent_to_constraint().p[1] );
    _double_spin_box_parent_to_constraint_z->setValue( _constraint->parent_to_constraint().p[2] );
    _double_spin_box_child_to_constraint_x->setValue( _constraint->child_to_constraint().p[0] );
    _double_spin_box_child_to_constraint_y->setValue( _constraint->child_to_constraint().p[1] );
    _double_spin_box_child_to_constraint_z->setValue( _constraint->child_to_constraint().p[2] );
  }

  QGroupBox * parent_group_box = new QGroupBox( "parent" );
  QGridLayout * parent_layout = new QGridLayout();
  parent_layout->addWidget( _combo_box_parent );
  parent_group_box->setLayout( parent_layout );

  QGroupBox * child_group_box = new QGroupBox( "child" );
  QGridLayout * child_layout = new QGridLayout();
  child_layout->addWidget( _combo_box_child );
  child_group_box->setLayout( child_layout );

  QGroupBox * type_group_box = new QGroupBox( "type" );
  QGridLayout * type_layout = new QGridLayout();
  type_layout->addWidget( _combo_box_type );
  type_group_box->setLayout( type_layout );

  QGroupBox * range_group_box = new QGroupBox( "range" );
  QGridLayout * range_layout = new QGridLayout();
  range_layout->addWidget( _push_button_x_min, 0, 0 );
  range_layout->addWidget( _double_spin_box_x_min, 0, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 0, 2 );
  range_layout->addWidget( new QLabel( QString( "x" ), this ), 0, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 0, 4 );
  range_layout->itemAtPosition( 0, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 0, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 0, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_x_max, 0, 5 );
  range_layout->addWidget( _push_button_x_max, 0, 6 );
  range_layout->addWidget( _push_button_y_min, 1, 0 );
  range_layout->addWidget( _double_spin_box_y_min, 1, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 1, 2 );
  range_layout->addWidget( new QLabel( QString( "y" ), this ), 1, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 1, 4 );
  range_layout->itemAtPosition( 1, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 1, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 1, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_y_max, 1, 5 );
  range_layout->addWidget( _push_button_y_max, 1, 6 );
  range_layout->addWidget( _push_button_z_min, 2, 0 );
  range_layout->addWidget( _double_spin_box_z_min, 2, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 2, 2 );
  range_layout->addWidget( new QLabel( QString( "z" ), this ), 2, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 2, 4 );
  range_layout->itemAtPosition( 2, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 2, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 2, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_z_max, 2, 5 );
  range_layout->addWidget( _push_button_z_max, 2, 6 );
  range_layout->addWidget( _push_button_roll_min, 3, 0 );
  range_layout->addWidget( _double_spin_box_roll_min, 3, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 3, 2 );
  range_layout->addWidget( new QLabel( QString( "roll" ), this ), 3, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 3, 4 );
  range_layout->itemAtPosition( 3, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 3, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 3, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_roll_max, 3, 5 );
  range_layout->addWidget( _push_button_roll_max, 3, 6 );
  range_layout->addWidget( _push_button_pitch_min, 4, 0 );
  range_layout->addWidget( _double_spin_box_pitch_min, 4, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 4, 2 );
  range_layout->addWidget( new QLabel( QString( "pitch" ), this ), 4, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 4, 4 );
  range_layout->itemAtPosition( 4, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 4, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 4, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_pitch_max, 4, 5 );
  range_layout->addWidget( _push_button_pitch_max, 4, 6 );
  range_layout->addWidget( _push_button_yaw_min, 5, 0 );
  range_layout->addWidget( _double_spin_box_yaw_min, 5, 1 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 5, 2 );
  range_layout->addWidget( new QLabel( QString( "yaw" ), this ), 5, 3 );
  range_layout->addWidget( new QLabel( QString::fromUtf8( "\u2264" ), this ), 5, 4 );
  range_layout->itemAtPosition( 5, 2 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 5, 3 )->setAlignment( Qt::AlignCenter );
  range_layout->itemAtPosition( 5, 4 )->setAlignment( Qt::AlignCenter );
  range_layout->addWidget( _double_spin_box_yaw_max, 5, 5 );
  range_layout->addWidget( _push_button_yaw_max, 5, 6 );
  range_group_box->setLayout( range_layout );

  QGroupBox * parent_to_constraint_group_box = new QGroupBox( "parent_to_constraint" );
  QGridLayout * parent_to_constraint_layout = new QGridLayout();
  parent_to_constraint_layout->addWidget( _double_spin_box_parent_to_constraint_x );
  parent_to_constraint_layout->addWidget( _double_spin_box_parent_to_constraint_y );
  parent_to_constraint_layout->addWidget( _double_spin_box_parent_to_constraint_z );
  parent_to_constraint_group_box->setLayout( parent_to_constraint_layout );

  QGroupBox * child_to_constraint_group_box = new QGroupBox( "child_to_constraint" );
  QGridLayout * child_to_constraint_layout = new QGridLayout();
  child_to_constraint_layout->addWidget( _double_spin_box_child_to_constraint_x );
  child_to_constraint_layout->addWidget( _double_spin_box_child_to_constraint_y );
  child_to_constraint_layout->addWidget( _double_spin_box_child_to_constraint_z );
  child_to_constraint_group_box->setLayout( child_to_constraint_layout );

  QGridLayout * widget_layout = new QGridLayout( this );
  widget_layout->addWidget( _label_id, 0, 0, 1, 2 );
  widget_layout->addWidget( parent_group_box, 1, 0, 1, 2 );
  widget_layout->addWidget( child_group_box, 2, 0, 1, 2 );
  widget_layout->addWidget( type_group_box, 3, 0, 1, 2 );
  widget_layout->addWidget( range_group_box, 4, 0, 1, 2 );
  widget_layout->addWidget( parent_to_constraint_group_box, 5, 0 );
  widget_layout->addWidget( child_to_constraint_group_box, 5, 1 );
  setLayout( widget_layout );

  connect( _combo_box_parent, SIGNAL( currentIndexChanged( int ) ), this, SLOT( _constraint_changed( int ) ) );
  connect( _combo_box_child, SIGNAL( currentIndexChanged( int ) ), this, SLOT( _constraint_changed( int ) ) );
  connect( _combo_box_type, SIGNAL( currentIndexChanged( int ) ), this, SLOT( _constraint_changed( int ) ) );

  connect( _push_button_x_min, SIGNAL( clicked() ), this, SLOT( _range_x_minimize() ) );
  connect( _push_button_x_max, SIGNAL( clicked() ), this, SLOT( _range_x_maximize() ) );
  connect( _push_button_y_min, SIGNAL( clicked() ), this, SLOT( _range_y_minimize() ) );
  connect( _push_button_y_max, SIGNAL( clicked() ), this, SLOT( _range_y_maximize() ) );
  connect( _push_button_z_min, SIGNAL( clicked() ), this, SLOT( _range_z_minimize() ) );
  connect( _push_button_z_max, SIGNAL( clicked() ), this, SLOT( _range_z_maximize() ) );
  connect( _push_button_roll_min, SIGNAL( clicked() ), this, SLOT( _range_roll_minimize() ) );
  connect( _push_button_roll_max, SIGNAL( clicked() ), this, SLOT( _range_roll_maximize() ) );
  connect( _push_button_pitch_min, SIGNAL( clicked() ), this, SLOT( _range_pitch_minimize() ) );
  connect( _push_button_pitch_max, SIGNAL( clicked() ), this, SLOT( _range_pitch_maximize() ) );
  connect( _push_button_yaw_min, SIGNAL( clicked() ), this, SLOT( _range_yaw_minimize() ) );
  connect( _push_button_yaw_max, SIGNAL( clicked() ), this, SLOT( _range_yaw_maximize() ) );

  connect( _double_spin_box_x_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_x_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_y_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_y_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_z_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_z_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_roll_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_roll_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_pitch_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_pitch_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_yaw_min, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_yaw_max, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );

  connect( _double_spin_box_parent_to_constraint_x, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_parent_to_constraint_y, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_parent_to_constraint_z, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  
  connect( _double_spin_box_child_to_constraint_x, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_child_to_constraint_y, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
  connect( _double_spin_box_child_to_constraint_z, SIGNAL( valueChanged( double ) ), this, SLOT( _constraint_changed( double ) ) );
}

Qt4_Widget_Constraint_Task_Space_Region_Editor::
~Qt4_Widget_Constraint_Task_Space_Region_Editor() {

}

Qt4_Widget_Constraint_Task_Space_Region_Editor::
Qt4_Widget_Constraint_Task_Space_Region_Editor( const Qt4_Widget_Constraint_Task_Space_Region_Editor& other )  : _robot_model( other._robot_model ),
    _robot_affordances( other._robot_affordances ),
    _object_affordances( other._object_affordances ){

}

Qt4_Widget_Constraint_Task_Space_Region_Editor&
Qt4_Widget_Constraint_Task_Space_Region_Editor::
operator=( const Qt4_Widget_Constraint_Task_Space_Region_Editor& other ) {

  return (*this);
}

void
Qt4_Widget_Constraint_Task_Space_Region_Editor::
_constraint_changed( void ){
  return;
}

void
Qt4_Widget_Constraint_Task_Space_Region_Editor::
_constraint_changed( double value ){
  _constraint->parent_to_constraint().p[0] = _double_spin_box_parent_to_constraint_x->value();
  _constraint->parent_to_constraint().p[1] = _double_spin_box_parent_to_constraint_y->value();
  _constraint->parent_to_constraint().p[2] = _double_spin_box_parent_to_constraint_z->value();

  _constraint->child_to_constraint().p[0] = _double_spin_box_child_to_constraint_x->value();
  _constraint->child_to_constraint().p[1] = _double_spin_box_child_to_constraint_y->value();
  _constraint->child_to_constraint().p[2] = _double_spin_box_child_to_constraint_z->value();
 
  _constraint->ranges()[0].first = _double_spin_box_x_min->value();
  _constraint->ranges()[0].second = _double_spin_box_x_max->value();
  _constraint->ranges()[1].first = _double_spin_box_y_min->value();
  _constraint->ranges()[1].second = _double_spin_box_y_max->value();
  _constraint->ranges()[2].first = _double_spin_box_z_min->value();
  _constraint->ranges()[2].second = _double_spin_box_z_max->value();
  _constraint->ranges()[3].first = _double_spin_box_roll_min->value();
  _constraint->ranges()[3].second = _double_spin_box_roll_max->value();
  _constraint->ranges()[4].first = _double_spin_box_pitch_min->value();
  _constraint->ranges()[4].second = _double_spin_box_pitch_max->value();
  _constraint->ranges()[5].first = _double_spin_box_yaw_min->value();
  _constraint->ranges()[5].second = _double_spin_box_yaw_max->value();
  return;
}

void
Qt4_Widget_Constraint_Task_Space_Region_Editor::
_constraint_changed( int index ){
  if( _combo_box_parent->currentIndex() < _robot_affordances.size() ){
    _constraint->parent() = _robot_affordances[ _combo_box_parent->currentIndex() ];
  }
  if( _combo_box_child->currentIndex() < _object_affordances.size() ){
    _constraint->child() = &( _object_affordances[ _combo_box_child->currentIndex() ] );
  }
  _constraint->contact_type() = ( contact_type_t )( _combo_box_type->currentIndex() );
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Constraint_Task_Space_Region_Editor& other ) {
    return out;
  }

}
