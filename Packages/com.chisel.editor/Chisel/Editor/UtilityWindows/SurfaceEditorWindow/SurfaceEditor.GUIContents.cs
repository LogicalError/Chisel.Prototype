using UnityEngine;

namespace Chisel.Editors
{
    internal partial class SurfaceEditor
    {
        private GUIStyle m_TabStyle        = null;
        private GUIStyle m_FooterStyle     = null;
        private GUIStyle m_FooterTextStyle = null;
        private GUIStyle m_MiniToggleStyle = null;
        private GUIStyle m_RightLabelStyle = null;

        private void InitStyles()
        {
            if( m_TabStyle == null )
                m_TabStyle = new GUIStyle( "ButtonMid" ) {};

            if( m_FooterStyle == null )
                m_FooterStyle = new GUIStyle( "DD Background" ) {};

            if( m_FooterTextStyle == null )
                m_FooterTextStyle = new GUIStyle( "ShurikenLabel" ) {};

            if( m_MiniToggleStyle == null )
                m_MiniToggleStyle = new GUIStyle( "OL Toggle" ) {};

            if( m_RightLabelStyle == null )
                m_RightLabelStyle = new GUIStyle( "Label" )
                {
                    alignment = TextAnchor.UpperRight,
                    normal    = new GUIStyleState() { textColor = new Color32( 255, 255, 255, 255 ) }
                };
        }
    }
}
