using System.Windows;
using Microsoft.Win32;

namespace RoboticArm.Services
{
    public sealed class WpfDialogService : IDialogService
    {
        private readonly Window? _owner;

        public WpfDialogService(Window? owner = null)
        {
            _owner = owner;
        }

        public void ShowMessage(string message)
        {
            if (_owner != null)
                MessageBox.Show(_owner, message);
            else
                MessageBox.Show(message);
        }

        public string? ShowOpenFile(string filter, string defaultName = "keyframes.json")
        {
            var dialog = new OpenFileDialog { Filter = filter, FileName = defaultName };
            return dialog.ShowDialog(_owner) == true ? dialog.FileName : null;
        }

        public string? ShowSaveFile(string filter, string defaultName = "keyframes.json")
        {
            var dialog = new SaveFileDialog { Filter = filter, FileName = defaultName };
            return dialog.ShowDialog(_owner) == true ? dialog.FileName : null;
        }
    }
}
