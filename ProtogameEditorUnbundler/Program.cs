using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using DomGen;
using Sce.Atf.Dom;

namespace ProtogameEditorUnbundler
{
    public static class Program
    {
        public static void Main(string[] args)
        {
            var sourceDirectory = new DirectoryInfo(Path.Combine(Environment.CurrentDirectory, "GameBundle"));
            var generatedDirectory = new DirectoryInfo(Path.Combine(Environment.CurrentDirectory, "GameBundle", "Generated"));

            sourceDirectory.Create();
            generatedDirectory.Create();

            // Delete old files.
            foreach (var file in generatedDirectory.GetFiles("*.cs"))
            {
                file.Delete();
            }
            foreach (var file in generatedDirectory.GetFiles("*.h"))
            {
                file.Delete();
            }
            foreach (var file in generatedDirectory.GetFiles("*.cpp"))
            {
                file.Delete();
            }

            using (var writer = new StreamWriter(Path.Combine(generatedDirectory.FullName, "_tempbundle.xsd")))
            {
                writer.Write(@"<?xml version=""1.0"" encoding=""utf-8"" ?>
<xs:schema
  elementFormDefault=""qualified""
  targetNamespace=""gap""
  xmlns=""gap""
  xmlns:xs=""http://www.w3.org/2001/XMLSchema"">

  <xs:include schemaLocation=""../Provided/gap.xsd""/>
  <xs:include schemaLocation=""../Provided/level_editor.xsd""/>
  " +
                             sourceDirectory.GetFiles("bundle.*.xsd")
                                 .Select(x => "<xs:include schemaLocation=\"../" + x.Name + "\"/>")
                                 .Aggregate((a, b) => a + b) + @"

</xs:schema>");
            }
            
            using (var writer = new StreamWriter(Path.Combine(generatedDirectory.FullName, "RegisterSchemaObjects.h")))
            {
                writer.Write(@"
//Copyright 2014 Sony Computer Entertainment America LLC. See License.txt.
#pragma once
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Bridge/GobBridge.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Core/Object.h""

#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/BillboardGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/BoxLightGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/ConeGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/ControlPointGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/CubeGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/CurveGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/CylinderGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/DirLightGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/GameLevel.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/GameObject.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/GameObjectGroup.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/Locator.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/OrcGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/PlaneGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/PointLightGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/PolyLineGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/SkyDome.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/SphereGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/TorusGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/GameObjectComponent.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/MeshComponent.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/SpinnerComponent.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/Terrain/TerrainGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/Terrain/TerrainMap.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/Terrain/LayerMap.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/Terrain/DecorationMap.h""

#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/Resource.h""

#include ""AutoGobs.h""

namespace LvEdEngine
{
    void InitGobBridge( GobBridge& bridge);
};


");
            }

            var bundle = generatedDirectory.GetFiles("_tempbundle.xsd").First();

            // C++ Schema Bundle
            DomGen.Program.Main(new[]
            {
                bundle.FullName,
                Path.Combine(bundle.DirectoryName, "RegisterSchemaObjects.cpp"),
                "LvEdEngine"
            });

            // C# Schema Bundle
            string inputFile = bundle.FullName;
            string outputFile = Path.Combine(bundle.DirectoryName, "LevelEditorSchema.cs");
            string codeNamespace = "LevelEditor";

            var typeLoader = new SchemaLoader();
            typeLoader.Load(inputFile);
            UTF8Encoding encoding = new UTF8Encoding();
            FileStream strm = File.Open(outputFile, FileMode.Create);

            string s = SchemaGen.Generate(typeLoader, "gap", codeNamespace, "Schema", new string[0]);
            byte[] bytes = encoding.GetBytes(s);
            strm.Write(bytes, 0, bytes.Length);

            var iconMappings = new Dictionary<string, byte[]>();

            // C++ Auto Gobs
            using (var implementation = new StreamWriter(Path.Combine(generatedDirectory.FullName, "AutoGobs.cpp")))
            {
                using (var header = new StreamWriter(Path.Combine(generatedDirectory.FullName, "AutoGobs.h")))
                {
                    header.WriteLine(@"
#pragma once
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/GameObject.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/GobSystem/PrimitiveShapeGob.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/Resource.h""

namespace LvEdEngine
{
");

                    implementation.WriteLine(@"

#include ""AutoGobs.h""
#include <algorithm>
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/RenderUtil.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/ShapeLib.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/RenderBuffer.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/Model.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/Renderer/TextureLib.h""
#include ""../../LevelEditorNativeRendering/LvEdRenderingEngine/ResourceManager/ResourceManager.h""

namespace LvEdEngine
{
");

                    foreach (var cls in typeLoader.GetNodeTypes())
                    {
                        // Process the class, and get the custom annotations of the class.
                        var annotations = (cls.GetTagLocal<IEnumerable<XmlNode>>() ?? new List<XmlNode>()).ToArray();

                        var protogameAnnotation = GetAnnotation(annotations, "Protogame.Info");
                        var nativeTypeAnnotation = GetAnnotation(annotations, "LeGe.NativeType");
                        string iconKey = null;
                        string modelRelativePath = null;

                        if (protogameAnnotation != null)
                        {
                            var iconData = protogameAnnotation.GetAttribute("IconData");
                            if (!string.IsNullOrWhiteSpace(iconData))
                            {
                                var data = Convert.FromBase64String(iconData);
                                var sha1 = new SHA1Managed();
                                var hash = sha1.ComputeHash(data);
                                var key = BitConverter.ToString(hash).Replace("-", "").ToLower();
                                iconMappings[key] = data;
                                iconKey = key;
                            }

                            modelRelativePath = protogameAnnotation.GetAttribute("ModelRelativePath");
                        }

                        if (protogameAnnotation != null && nativeTypeAnnotation != null)
                        {
                            var nativeTypeName = GetAttributeOnAnnotation(nativeTypeAnnotation, "nativeName");
                            if (nativeTypeName != null)
                            {
                                var hasCustomNativeDefinition = GetBooleanAttributeOnAnnotation(protogameAnnotation,
                                    "HasCustomNativeDefinition", false);
                                if (!hasCustomNativeDefinition)
                                {
                                    Console.WriteLine("Generating C++ autogob for " + nativeTypeName + "...");
                                    AddHeader(header, annotations, protogameAnnotation, nativeTypeAnnotation,
                                        nativeTypeName);
                                    AddImplementation(implementation, annotations, protogameAnnotation, nativeTypeAnnotation,
                                        nativeTypeName, iconKey, modelRelativePath);
                                }
                            }
                        }
                    }

                    header.WriteLine(@"
}; // namespace
");

                    implementation.WriteLine(@"
}; // namespace
");
                }
            }

            // C++ Resource Files
            using (var resource = new StreamWriter(Path.Combine(generatedDirectory.FullName, "resource.rc")))
            {
                resource.WriteLine(@"
Lighting.shh             SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\Lighting.shh""
BasicShader.hlsl         SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\BasicShader.hlsl""
BasicRenderer.hlsl       SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\BasicRenderer.hlsl""
Billboard.hlsl           SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\Billboard.hlsl""
TexturedShader.hlsl      SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\TexturedShader.hlsl""
WireframeShader.hlsl     SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\WireframeShader.hlsl""
SkyDome.hlsl             SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\SkyDome.hlsl""
SkySphere.hlsl           SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\SkySphere.hlsl""
FontShader.hlsl          SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\FontShader.hlsl""
ShadowMapGen.hlsl        SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\ShadowMapGen.hlsl""
LineShader.hlsl          SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\LineShader.hlsl""
TerrainShader.hlsl       SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\TerrainShader.hlsl""
NormalsShader.hlsl       SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\NormalsShader.hlsl""
SolidWireframeHelper.shh SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\SolidWireframeHelper.shh""
Fog.shh                  SHADER       ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Shaders\\Fog.shh""

// Icon by http://iconleak.com/works/free-app-icons/
Light.png           Texture    ""..\\..\\LevelEditorNativeRendering\\LvEdRenderingEngine\\Resources\\Light.png""

");

                foreach (var kv in iconMappings)
                {
                    using (var stream = new FileStream(Path.Combine(generatedDirectory.FullName, kv.Key + ".png"), FileMode.Create, FileAccess.Write))
                    {
                        stream.Write(kv.Value, 0, kv.Value.Length);

                        resource.WriteLine(kv.Key + ".png       Texture       \"" + Path.Combine(generatedDirectory.FullName, kv.Key + ".png").Replace("\\","\\\\") + "\"");
                    }
                }
            }

            // C++ Macros
            using (var macros = new StreamWriter(Path.Combine(generatedDirectory.FullName, "Macros.h")))
            {
                macros.Write(@"#pragma once

#define LOAD_TEXTURES() \
    LoadEmbeddedTexture(device, L""Light.png""); \
");

                foreach (var kv in iconMappings)
                {
                    macros.WriteLine(@"    LoadEmbeddedTexture(device, L""" + kv.Key + @".png""); \");
                }

                macros.Write(@"    

    ");
            }
        }

        private static void AddImplementation(StreamWriter implementation, XmlNode[] annotations, XmlElement protogameAnnotation, XmlElement nativeTypeAnnotation, string nativeTypeName, string iconKey, string modelRelativePath)
        {
            var excludeImmediateProperties = new string[0];

            var nativePropertyDeclarations = new List<string>();
            var nativeFieldDeclarations = new List<string>();
            var nativeMethodDeclarations = new List<string>();

            if (protogameAnnotation.GetAttribute("RenderMode") == "PrimitiveShape")
            {
                excludeImmediateProperties = new[]
                {
                    "color",
                    "emissive",
                    "specular",
                    "specularPower",
                    "diffuse",
                    "normal",
                    "textureTransform"
                };
            }
            else if (protogameAnnotation.GetAttribute("RenderMode") == "Icon")
            {
                nativeMethodDeclarations.Add(string.Format(@"
{0}::{0}()
{{
    SetCastsShadows( false );       // doesn't block the light
    SetReceivesShadows( false );    // no shadow is cast on it        
    m__Mesh = ShapeLibGetMesh( RenderShape::Quad); 
    m_localBounds = AABB(float3(-0.5f,-0.5f,-0.5f), float3(0.5f,0.5f,0.5f));
}}

void {0}::GetRenderables(RenderableNodeCollector* collector, RenderContext* context)
{{
	if (!IsVisible(context->Cam().GetFrustum()))
		return;
    
	super::GetRenderables(collector, context);

    RenderableNode renderable;
    GameObject::SetupRenderable(&renderable,context);
    renderable.mesh = m__Mesh;
    renderable.textures[TextureType::DIFFUSE] =  TextureLib::Inst()->GetByName(L""{1}.png"");

    float3 objectPos = &m_world.M41;
    Camera & cam = context->Cam();
    Matrix billboard = Matrix::CreateBillboard(objectPos, cam.CamPos(), cam.CamUp(), cam.CamLook());
    float sx = length(float3(&m_local.M11));
    float sy = length(float3(&m_local.M21));
    float sz = length(float3(&m_local.M31));
    Matrix scale = Matrix::CreateScale(sx, sy, sz);
    renderable.WorldXform = scale * billboard;

    RenderFlagsEnum flags = RenderFlags::Textured;
    collector->Add(renderable, flags, Shaders::BillboardShader);
}}
            ",
                nativeTypeName,
                iconKey));
            }
            else if (protogameAnnotation.GetAttribute("RenderMode") == "Model")
            {
                nativeMethodDeclarations.Add(string.Format(@"
{0}::{0}()
{{
    m_resource = NULL;
}}

{0}::~{0}()
{{
    SAFE_DELETE(m_resource);
}}

void {0}::GetRenderables(RenderableNodeCollector* collector, RenderContext* context)
{{
	if (!IsVisible(context->Cam().GetFrustum()))
		return;

    if (m_resource == NULL) {{
        m_resource = new ResourceReference();

		std::wstring path = ResourceManager::Inst()->m_ResourceRoot;
		path += L""{1}"";

        m_resource->SetTarget(path.c_str());
    }}

	super::GetRenderables(collector, context);

    RenderFlagsEnum flags = (RenderFlagsEnum)(RenderFlags::Textured | RenderFlags::Lit);
    collector->Add( m_renderables.begin(), m_renderables.end(), flags, Shaders::TexturedShader );
}}

void {0}::BuildRenderables()
{{
    m_renderables.clear();
    Model* model = NULL;
    assert(m_resource);
    model = (Model*)m_resource->GetTarget();
        
    assert(model && model->IsReady());

    const NodeDict& nodes = model->Nodes();
    for(auto nodeIt = nodes.begin(); nodeIt != nodes.end(); ++nodeIt)
    {{
        Node* node = nodeIt->second;
        assert(m_modelTransforms.size() >= node->index);
        const Matrix& world = m_modelTransforms[node->index]; // transform array holds world matricies already, not local
        for(auto geoIt = node->geometries.begin(); geoIt != node->geometries.end(); ++geoIt)
        {{
            Geometry* geo = (*geoIt);
            Material* mat = geo->material;
            RenderableNode renderNode;
            renderNode.mesh = geo->mesh;
            renderNode.WorldXform = world;
            renderNode.bounds = geo->mesh->bounds;
            renderNode.bounds.Transform(renderNode.WorldXform);
            renderNode.objectId = GetInstanceId();
            renderNode.diffuse =  mat->diffuse;
            renderNode.specular = mat->specular.xyz();
            renderNode.specPower = mat->power;
            renderNode.SetFlag( RenderableNode::kShadowCaster, GetCastsShadows() );
            renderNode.SetFlag( RenderableNode::kShadowReceiver, GetReceivesShadows() );

            LightingState::Inst()->UpdateLightEnvironment(renderNode);

            for(unsigned int i = TextureType::MIN; i < TextureType::MAX; ++i)
            {{
                renderNode.textures[i] = geo->material->textures[i];
            }}
            m_renderables.push_back(renderNode);
        }}
    }}
}}

void {0}::AddResource(ResourceReference* r, int /*index*/)
{{
    m_resource = r;
    m_modelTransforms.clear();
    m_renderables.clear();
    InvalidateBounds();
    InvalidateWorld();        
}}

void {0}::RemoveResource(ResourceReference* /*r*/)
{{
    AddResource(NULL, -1);
}}

void {0}::Update(const FrameTime& fr, UpdateTypeEnum updateType)
{{
    super::Update(fr,updateType);
	bool updatedBound = m_worldXformUpdated;

    Model* model = m_resource ? (Model*)m_resource->GetTarget() : NULL;                     
    if( model && model->IsReady())
    {{
        if(m_modelTransforms.empty() || m_worldXformUpdated)
        {{
                const MatrixList& matrices = model->AbsoluteTransforms();
                m_modelTransforms.resize(matrices.size());
                for( unsigned int i = 0; i < m_modelTransforms.size(); ++i)
                {{
                    m_modelTransforms[i] = matrices[i] * m_world; // transform matrix array now holds complete world transform.
                }}
                BuildRenderables(); 
				updatedBound = true;
        }}
    }}

    m_boundsDirty = updatedBound;
    if(m_boundsDirty)        
    {{
        if(!m_modelTransforms.empty())
        {{
            // assert(model && model->IsReady());
            m_localBounds = model->GetBounds();                                
            if(m_parent) m_parent->InvalidateBounds();                
        }}
        else
        {{
            m_localBounds = AABB(float3(-0.5f,-0.5f,-0.5f), float3(0.5f,0.5f,0.5f));                
        }}
        this->UpdateWorldAABB();            
    }}

    if(RenderContext::Inst()->LightEnvDirty)
    {{
        // update light env.
        for(auto  renderNode = m_renderables.begin(); renderNode != m_renderables.end(); renderNode++)
        {{
            LightingState::Inst()->UpdateLightEnvironment(*renderNode);
        }}
    }}
}}
            ",
                nativeTypeName,
                modelRelativePath.Replace("\\","\\\\")));
            }

            foreach (var prop in annotations.OfType<XmlElement>().Where(x => x.LocalName == "LeGe.NativeProperty").ToArray())
            {
                if (excludeImmediateProperties.Contains(prop.GetAttribute("name")))
                {
                    // We know the base type provides this.
                    continue;
                }
                
                var nativeName = prop.GetAttribute("nativeName");
                var nativeType = prop.GetAttribute("nativeType");

                var access = prop.GetAttribute("access").Split(',');
                if (access.Contains("set"))
                {
                    nativePropertyDeclarations.Add(string.Format(@"
void {0}::Set{1}({2} v)
{{
    m_{1} = v;
}}

", 
                        nativeTypeName,
                        nativeName,
                        nativeType));
                }
            }

            implementation.WriteLine(
                nativePropertyDeclarations.Concat(nativeFieldDeclarations).Concat(nativeMethodDeclarations).DefaultIfEmpty(string.Empty).Aggregate((a, b) => a + b));
        }

        private static void AddHeader(StreamWriter header, XmlNode[] annotations, XmlElement protogameAnnotation, XmlElement nativeTypeAnnotation, string nativeTypeName)
        {
            var baseType = "GameObject";
            var baseConstructor = string.Empty;
            var excludeImmediateProperties = new string[0];

            var nativePropertyDeclarations = new List<string>();
            var nativeFieldDeclarations = new List<string>();
            var nativeMethodDeclarations = new List<string>();
            var protectedDeclarations = new List<string>();

            if (protogameAnnotation.GetAttribute("RenderMode") == "PrimitiveShape")
            {
                baseType = "PrimitiveShapeGob";
                baseConstructor = " : PrimitiveShapeGob( RenderShape::" + protogameAnnotation.GetAttribute("PrimitiveShape") + " )";
                excludeImmediateProperties = new[]
                {
                    "color",
                    "emissive",
                    "specular",
                    "specularPower",
                    "diffuse",
                    "normal",
                    "textureTransform"
                };
            }
            else if (protogameAnnotation.GetAttribute("RenderMode") == "Icon")
            {
                nativeMethodDeclarations.Add("virtual void GetRenderables(RenderableNodeCollector* collector, RenderContext* context);");
                nativeFieldDeclarations.Add("Mesh* m__Mesh;");
            }
            else if (protogameAnnotation.GetAttribute("RenderMode") == "Model")
            {
                nativeMethodDeclarations.Add("virtual void GetRenderables(RenderableNodeCollector* collector, RenderContext* context);");
                nativeMethodDeclarations.Add("void AddResource(ResourceReference * r, int index);");
                nativeMethodDeclarations.Add("void RemoveResource(ResourceReference * r);");
                nativeMethodDeclarations.Add("void Update(const FrameTime& fr, UpdateTypeEnum updateType);");
                nativeMethodDeclarations.Add("virtual ~" + nativeTypeName + "();");
                protectedDeclarations.Add("void BuildRenderables();");
                protectedDeclarations.Add("ResourceReference* m_resource;");
                protectedDeclarations.Add("std::vector<Matrix> m_modelTransforms;");
                protectedDeclarations.Add("RenderNodeList m_renderables;");
            }

            foreach (var prop in annotations.OfType<XmlElement>().Where(x => x.LocalName == "LeGe.NativeProperty").ToArray())
            {
                if (excludeImmediateProperties.Contains(prop.GetAttribute("name")))
                {
                    // We know the base type provides this.
                    continue;
                }

                var name = prop.GetAttribute("name");
                var nativeName = prop.GetAttribute("nativeName");
                var nativeType = prop.GetAttribute("nativeType");

                var access = prop.GetAttribute("access").Split(',');
                if (access.Contains("set"))
                {
                    nativePropertyDeclarations.Add(string.Format("void Set{1}({0} v);", nativeType, nativeName));
                }
                if (access.Contains("get"))
                {
                    nativePropertyDeclarations.Add(string.Format("{0} Get{1}(){{return m_{1}}};", nativeType, nativeName));
                }
                nativeFieldDeclarations.Add(string.Format("{0} m_{1};", nativeType, nativeName));
            }
            
            header.WriteLine(@"
    class {0} : public {1}
    {{
    public:
        {0}() {2} {5}
        virtual const char* ClassName() const {{return StaticClassName();}}
        static const char* StaticClassName(){{return ""{0}"";}}

        {3}
    protected:
        {6}
    private:
        {4}

        typedef {1} super;
    }};
", 
                new object[] {
                    nativeTypeName,
                    baseType,
                    baseConstructor,
                    nativePropertyDeclarations.Concat(nativeMethodDeclarations).DefaultIfEmpty(string.Empty).Aggregate((a, b) => a + b),
                    nativeFieldDeclarations.DefaultIfEmpty(string.Empty).Aggregate((a, b) => a + b),
                    baseType == "GameObject" ? ";" : "{{}}",
                    protectedDeclarations.DefaultIfEmpty(string.Empty).Aggregate((a, b) => a + b),
                });
        }

        #region Xml Utilities

        private static bool GetBooleanAttributeOnAnnotation(XmlElement annotation, string name, bool @default)
        {
            if (annotation == null)
            {
                return @default;
            }
            
            var attribute = annotation.Attributes.OfType<XmlAttribute>().FirstOrDefault(x => x.LocalName == name);
            if (attribute == null)
            {
                return @default;
            }

            if (@default)
            {
                return attribute.Value == "false";
            }
            else
            {
                return attribute.Value == "true";
            }
        }

        private static string GetAttributeOnAnnotation(XmlElement annotation, string name)
        {
            if (annotation == null)
            {
                return null;
            }

            var attribute = annotation.Attributes.OfType<XmlAttribute>().FirstOrDefault(x => x.LocalName == name);
            if (attribute == null)
            {
                return null;
            }

            return attribute.Value;
        }

        private static XmlElement GetAnnotation(IEnumerable<XmlNode> annotations, string name)
        {
            if (annotations == null)
            {
                return null;
            }

            return annotations.OfType<XmlElement>().FirstOrDefault(x => x.LocalName == name);
        }

        #endregion
    }
}
