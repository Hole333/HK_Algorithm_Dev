using VM.Core;

/// <summary>
/// 命名空间规则名称：模块名+Cs，生成的类库与命名空间同名
/// </summary>
namespace DEBUG_DETECT_LINECs
{
    /// <summary>
    /// 界面工具类名称：模块名+Tool
    /// </summary>
    public class DEBUG_DETECT_LINETool : VmModule
    {
        public DEBUG_DETECT_LINETool()
        {
            
        }
        /// <summary>
        /// 构造中要传入模块名称
        /// </summary>
        /// <param name="param"></param>
        public DEBUG_DETECT_LINETool(object param) : base(param, "DEBUG_DETECT_LINE")
        {
            
        }
    }
}
