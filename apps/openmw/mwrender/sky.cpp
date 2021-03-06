#include "sky.hpp"
#include "Caelum.h"

namespace MWRender
{
    //
    // Implements a Caelum sky with default settings.
    //
    // Note: this is intended as a temporary solution to provide some form of 
    // sky rendering.  This code will obviously need significant tailoring to
    // support fidelity with Morrowind's rendering.  Before doing major work
    // on this class, more research should be done to determine whether
    // Caelum or another plug-in such as SkyX would be best for the long-term.
    //
    class CaelumManager : public SkyManager
    {
    protected:
        Caelum::CaelumSystem*   mpCaelumSystem;

    public:
                 CaelumManager (Ogre::RenderWindow* pRenderWindow, 
                                   Ogre::Camera* pCamera,
                                   const boost::filesystem::path& resDir);
        virtual ~CaelumManager ();
        
        virtual void enable() {}
        
        virtual void disable() {}
        
        virtual void setHour (double hour) {}
        ///< will be called even when sky is disabled.
        
        virtual void setDate (int day, int month) {}
        ///< will be called even when sky is disabled.
        
        virtual int getMasserPhase() const { return 0; }
        ///< 0 new moon, 1 waxing or waning cresecent, 2 waxing or waning half,
        /// 3 waxing or waning gibbous, 4 full moon
        
        virtual int getSecundaPhase() const { return 0; }
        ///< 0 new moon, 1 waxing or waning cresecent, 2 waxing or waning half,
        /// 3 waxing or waning gibbous, 4 full moon
        
        virtual void setMoonColour (bool red) {}
    };

    CaelumManager::CaelumManager (Ogre::RenderWindow* pRenderWindow, 
                                  Ogre::Camera* pCamera,
                                  const boost::filesystem::path& resDir)
        : mpCaelumSystem        (NULL)
    {
        using namespace Ogre;
        using namespace Caelum;

        assert(pCamera);
        assert(pRenderWindow);

        // Load the Caelum resources
        //
        ResourceGroupManager::getSingleton().addResourceLocation((resDir / "caelum").string(), "FileSystem", "Caelum");
        ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

        // Load the Caelum resources
        //
        Ogre::SceneManager* pScene = pCamera->getSceneManager();
        Caelum::CaelumSystem::CaelumComponent componentMask = CaelumSystem::CAELUM_COMPONENTS_DEFAULT;
        mpCaelumSystem = new Caelum::CaelumSystem (Root::getSingletonPtr(), pScene, componentMask);
        
        // Set time acceleration.
        mpCaelumSystem->getUniversalClock()->setTimeScale(128);       

        // Disable fog since OpenMW is handling OGRE fog elsewhere
        mpCaelumSystem->setManageSceneFog(false);

        // Change the camera far distance to make sure the sky is not clipped
        pCamera->setFarClipDistance(50000);

        // Register Caelum as an OGRE listener
        pRenderWindow->addListener(mpCaelumSystem);
        Root::getSingletonPtr()->addFrameListener(mpCaelumSystem);
    }

    CaelumManager::~CaelumManager() 
    {
        if (mpCaelumSystem) 
            mpCaelumSystem->shutdown (false);
    }

    /// Creates and connects the sky rendering component to OGRE.
    ///
    /// \return NULL on failure.
    /// 
    SkyManager* SkyManager::create (Ogre::RenderWindow* pRenderWindow, 
                                    Ogre::Camera*       pCamera,
                                    const boost::filesystem::path& resDir)
    {
        SkyManager* pSkyManager = NULL;

        try
        {
            pSkyManager = new CaelumManager(pRenderWindow, pCamera, resDir);
        }
        catch (Ogre::Exception& e)
        {
            std::cout << "\nOGRE Exception when attempting to add sky: " 
                << e.getFullDescription().c_str() << std::endl;
        }
        catch (std::exception& e)
        {
            std::cout << "\nException when attempting to add sky: " 
                << e.what() << std::endl;
        }

        return pSkyManager;
    }
} 
