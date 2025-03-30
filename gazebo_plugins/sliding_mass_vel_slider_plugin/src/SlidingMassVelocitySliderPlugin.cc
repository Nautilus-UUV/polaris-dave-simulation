#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <QtGui/QSlider>
#include <QtWidgets/QWidget>

class SlidingMassVelocitySliderPlugin : public gazebo::ModelPlugin
{
public:
    SlidingMassVelocitySliderPlugin() {}

    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Create the slider for controlling velocity
        slider = new QSlider();
        slider->setOrientation(Qt::Horizontal);
        slider->setRange(-100, 100);
        slider->setValue(0);  // Start at 0 velocity

        // Connect slider value to velocity command
        connect(slider, &QSlider::valueChanged, this, &SlidingMassVelocitySliderPlugin::OnSliderValueChanged);

        // Setup Gazebo transport and publisher for sliding mass velocity command
        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();
        publisher = node->Advertise<gazebo::msgs::Double>("~/model/sliding_mass_joint/cmd_vel");

        // Optional: Add the slider to a Qt window in the Gazebo GUI
        QWidget *window = new QWidget();
        window->setWindowTitle("Sliding Mass Velocity Control");
        window->show();
    }

private:
    void OnSliderValueChanged(int value)
    {
        // Convert slider value to sliding mass velocity and publish
        gazebo::msgs::Double msg;
        msg.set_data(static_cast<double>(value) * 0.01);  // Scale the slider value to your desired range
        publisher->Publish(msg);
    }

private:
    QSlider* slider;
    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr publisher;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SlidingMassVelocitySliderPlugin)

