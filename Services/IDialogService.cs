namespace RoboticArm.Services
{
    public interface IDialogService
    {
        void ShowMessage(string message);
        string? ShowOpenFile(string filter, string defaultName = "keyframes.json");
        string? ShowSaveFile(string filter, string defaultName = "keyframes.json");
    }
}
